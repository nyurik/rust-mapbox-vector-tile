use std::collections::HashMap;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufWriter;
use std::rc::Rc;

use flate2::read::GzDecoder;
use flate2::write::GzEncoder;
use flate2::Compression;
use geo::orient::{Direction, Orient};
use geo::winding_order::{Winding, WindingOrder};
use geo::{Coord, Geometry, LineString, MultiLineString, MultiPoint, MultiPolygon, Point, Polygon};
use protobuf::Message;
use serde_derive::Serialize;

mod proto {
    include!(concat!(env!("OUT_DIR"), "/mod.rs"));
}
use proto::vector_tile;

use crate::Error::*;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Drawing commands are invalid. There are {} commands, and it must be a multiple of 3: {0:?}", (.0).0.len())]
    InvalidDrawingCommands(DrawingCommands),

    #[error("Invalid number of points for a polygon, got {0} and we need > 1: cmds: {1:?}")]
    InvalidPolygonPointCount(usize, Vec<DrawingCommand>),

    #[error("No valid polygons created")]
    NoValidPolygons,

    #[error("No valid rings created")]
    NoValidRings,

    #[error("Expecting a LineTo command, got a {:?}. All cmds {0:?}", .0[1])]
    UnexpectedDrawingCommand(Vec<DrawingCommand>),

    #[error("IO Error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Protobuf parsing error: {0}")]
    Pbf(#[from] protobuf::Error),
}

/// What should be done if, when parsing a file, there's an invalid geometry, i.e. the MVT file is
/// invalid
#[derive(Debug, PartialEq, Serialize, Clone, Copy)]
pub enum InvalidGeometryTactic {
    /// If one geom is invalid, the whole tile is viewed as invalid and not returned.
    StopProcessing,

    /// The feature with invalid geometry is ignored and skipped. The rest of the features in the
    /// file (with valid geometries) are included.
    DropBrokenFeature,
}

#[derive(Debug, PartialEq, Clone, Default)]
pub struct Properties(pub HashMap<Rc<String>, Value>);

impl Properties {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert<K: Into<Rc<String>>, V: Into<Value>>(&mut self, k: K, v: V) {
        self.0.insert(k.into(), v.into());
    }

    pub fn set<K: Into<Rc<String>>, V: Into<Value>>(mut self, k: K, v: V) -> Self {
        self.insert(k.into(), v.into());
        self
    }
}

/// A single feature. It has a geometry and some properties (i.e. tags)
#[derive(Debug, PartialEq, Clone)]
pub struct Feature {
    /// The geometry
    pub geometry: Geometry<i32>,

    /// The properties. Uses an `Rc` because properties can be shared between tiles.
    pub properties: Rc<Properties>,
}

impl Feature {
    /// Create a Feature with this `geometry` and these `properties`
    pub fn new(geometry: Geometry<i32>, properties: Rc<Properties>) -> Self {
        Self {
            geometry,
            properties,
        }
    }

    /// Create a feature (with no properties) from this geometry.
    pub fn from_geometry(geometry: Geometry<i32>) -> Self {
        Self::new(geometry, Rc::new(Properties::new()))
    }

    pub fn get_point(&self) -> Option<Point<i32>> {
        match self.geometry {
            Geometry::Point(p) => Some(p),
            _ => None,
        }
    }

    pub fn translate_geometry(&mut self, x_func: &dyn Fn(i32) -> i32, y_func: &dyn Fn(i32) -> i32) {
        // FIXME why the back and forth and not just set it
        self.geometry = match self.geometry {
            Geometry::Point(mut p) => {
                let x = x_func(p.x());
                let y = y_func(p.y());
                p.set_x(x).set_y(y);
                Geometry::Point(p)
            }
            _ => unimplemented!(),
        };
    }

    /// Sets a property for this feature. It will panic if the properties are shared with other
    /// features. Don't call this if that could happen.
    /// Consumes the feature and returns it. Useful for Building pattern
    pub fn set<K: Into<Rc<String>>, V: Into<Value>>(mut self, k: K, v: V) -> Self {
        Rc::get_mut(&mut self.properties).unwrap().insert(k, v);
        self
    }
}

/// Convert a geometry to a feature.
impl From<Geometry<i32>> for Feature {
    fn from(geom: Geometry<i32>) -> Self {
        Self::from_geometry(geom)
    }
}

/// A Layer in a vector tile
#[derive(Debug, PartialEq, Clone)]
pub struct Layer {
    /// The layer's name
    pub name: String,

    /// The features in this layer
    pub features: Vec<Feature>,

    /// The "extent" of this layer. Usualyy 4096 is used.
    pub extent: u32,
}

impl Layer {
    /// Create an empty layer with this name (and 4096 extent)
    pub fn new<S: Into<String>>(name: S) -> Self {
        Self {
            name: name.into(),
            features: Vec::new(),
            extent: 4096,
        }
    }

    /// Construct layer with this name and extent
    pub fn new_and_extent(name: String, extent: u32) -> Self {
        Self {
            name,
            features: Vec::new(),
            extent,
        }
    }

    /// Move all the geometries in this layer so that it's at this `geometry_tile`.
    pub fn set_locations(&mut self, geometry_tile: &slippy_map_tiles::Tile) {
        let extent = self.extent as i32;
        let top = geometry_tile.top() as i32;
        let bottom = geometry_tile.bottom() as i32;
        let left = geometry_tile.left() as i32;
        let right = geometry_tile.right() as i32;
        let width = right - left;
        let height = bottom - top;

        let x_func = |x: i32| left + (x / extent) * width;
        let y_func = |y: i32| top + (y / extent) * height;

        for f in &mut self.features {
            f.translate_geometry(&x_func, &y_func);
        }
    }

    /// Add a feature to this layer.
    pub fn add_feature(&mut self, f: Feature) {
        // TODO error checking for non-integer geom?
        self.features.push(f);
    }

    /// True iff this layer has no features
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }

    /// Encode this layer to the `writer`.
    pub fn write_to<W: Write>(self, writer: &mut W) {
        if self.is_empty() {
            return;
        }

        let converted: vector_tile::tile::Layer = self.into();
        let mut cos = protobuf::CodedOutputStream::new(writer);
        converted.write_to(&mut cos).unwrap();
        cos.flush().unwrap();
    }

    /// Encode this layer into bytes.
    pub fn to_bytes(self) -> Vec<u8> {
        let mut res = Vec::new();
        self.write_to(&mut res);
        res
    }
}

// TODO probably some way to merge these
impl From<String> for Layer {
    fn from(name: String) -> Self {
        Self::new(name)
    }
}
impl<'a> From<&'a str> for Layer {
    fn from(name: &str) -> Self {
        Self::new(name.to_string())
    }
}

/// Internally used to keep track of tags/values
#[derive(Debug)]
struct TagIndex<T>
where
    T: Sized,
{
    _data: Vec<(usize, T)>,
}

impl<T> TagIndex<T>
where
    T: Sized + PartialEq + ToOwned + Clone,
{
    fn new() -> Self {
        Self { _data: Vec::new() }
    }

    fn add_item(&mut self, item: &T) {
        match self
            ._data
            .iter()
            .enumerate()
            .filter(|&(_i, &(_num, ref val))| val == item)
            .take(1)
            .next()
        {
            None => {
                // not found, insert at end
                self._data.push((1, item.clone()));
            }
            Some((i, _)) => {
                // it's at position i
                self._data[i].0 += 1;

                // bubble it up so that it's all in order
                let mut i = i;
                loop {
                    if i == 0 {
                        break;
                    }
                    if self._data[i].0 > self._data[i - 1].0 {
                        self._data.swap(i, i - 1);
                    } else {
                        // Everything is OK now
                        break;
                    }
                    i -= 1;
                }
            }
        }
    }

    fn index_for_item(&self, item: &T) -> usize {
        self._data
            .iter()
            .enumerate()
            .filter_map(|(i, &(_num, ref val))| if val == item { Some(i) } else { None })
            .take(1)
            .next()
            .unwrap()
    }

    // FIXME getting warnings about lifetime for T
    //fn into_items(self) ->  {
    //    self._data.into_iter().map(|x| x.1)
    //}
}

fn create_indexes(features: &[Feature]) -> (TagIndex<String>, TagIndex<Value>) {
    let mut keys: TagIndex<String> = TagIndex::new();
    let mut values: TagIndex<Value> = TagIndex::new();

    for f in features.iter() {
        for (k, v) in &f.properties.0 {
            keys.add_item(k);
            values.add_item(v);
        }
    }

    (keys, values)
}

impl From<Layer> for vector_tile::tile::Layer {
    fn from(v: Layer) -> Self {
        let mut res = Self::new();

        let Layer {
            name,
            features,
            extent,
        } = v;

        res.set_name(name);
        res.set_version(2);
        res.set_extent(extent);

        let (keys, values) = create_indexes(&features);

        // There's lots of 'cannot move out of borrowed context' errors here
        for f in features {
            let mut pbf_feature = vector_tile::tile::Feature::new();
            for (k, v) in &f.properties.0 {
                pbf_feature.tags.push(keys.index_for_item(k) as u32);
                pbf_feature.tags.push(values.index_for_item(v) as u32);
            }
            let geom: DrawingCommands = (&f.geometry).into();
            pbf_feature.geometry = geom.into();

            pbf_feature.set_type(match f.geometry {
                Geometry::Point(_) => vector_tile::tile::GeomType::POINT,
                Geometry::MultiPoint(_) => vector_tile::tile::GeomType::POINT,
                Geometry::LineString(_) => vector_tile::tile::GeomType::LINESTRING,
                Geometry::MultiLineString(_) => vector_tile::tile::GeomType::LINESTRING,
                Geometry::Polygon(_) => vector_tile::tile::GeomType::POLYGON,
                Geometry::MultiPolygon(_) => vector_tile::tile::GeomType::POLYGON,
                _ => vector_tile::tile::GeomType::UNKNOWN,
            });

            res.features.push(pbf_feature);
        }

        // set the keys & values
        for k in keys._data.into_iter().map(|x| x.1) {
            res.keys.push(k)
        }
        for v in values._data.into_iter().map(|x| x.1) {
            res.values.push(v.into())
        }

        res
    }
}

/// One Vector Tile.
#[derive(Debug, PartialEq, Clone, Default)]
pub struct Tile {
    /// The layers in this vector tile
    pub layers: Vec<Layer>,
}

impl Tile {
    /// Construct an empty layer
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a layer to this tile, at the end.
    pub fn add_layer<L: Into<Layer>>(&mut self, l: L) {
        let l = l.into();
        // TODO check for duplicate layer name
        self.layers.push(l);
    }

    /// Read compressed .mvt file and parse it
    pub fn from_file(filename: &str) -> Result<Self, Error> {
        let mut file = File::open(filename)?;
        let mut contents: Vec<u8> = Vec::new();
        file.read_to_end(&mut contents)?;
        // FIXME if the gzip file is empty then return something sensible

        Self::from_compressed_bytes(&contents)
    }

    /// Try to parse a VT from some (gzip) compressed bytes
    pub fn from_compressed_bytes(bytes: &[u8]) -> Result<Self, Error> {
        Self::from_compressed_bytes_with_tactic(bytes, InvalidGeometryTactic::StopProcessing)
    }

    pub fn from_compressed_bytes_with_tactic(
        bytes: &[u8],
        invalid_geom_tactic: InvalidGeometryTactic,
    ) -> Result<Self, Error> {
        let mut decompressor = GzDecoder::new(bytes);
        let mut contents: Vec<u8> = Vec::new();
        decompressor.read_to_end(&mut contents).unwrap();

        Self::from_uncompressed_bytes_with_tactic(&contents, invalid_geom_tactic)
    }

    /// Try to parse a VT from some uncompressed bytes. i.e. raw protobuf
    pub fn from_uncompressed_bytes(bytes: &[u8]) -> Result<Self, Error> {
        Self::from_uncompressed_bytes_with_tactic(bytes, InvalidGeometryTactic::StopProcessing)
    }

    pub fn from_uncompressed_bytes_with_tactic(
        bytes: &[u8],
        invalid_geom_tactic: InvalidGeometryTactic,
    ) -> Result<Self, Error> {
        let tile = vector_tile::Tile::parse_from_bytes(bytes)?;
        pbftile_to_tile(tile, invalid_geom_tactic)
    }

    /// Construct a tile from some layers
    pub fn from_layers(layers: Vec<Layer>) -> Self {
        Self { layers }
    }

    pub fn set_locations(&mut self, geometry_tile: &slippy_map_tiles::Tile) {
        for l in &mut self.layers {
            l.set_locations(geometry_tile);
        }
    }

    pub fn add_feature(&mut self, layer_name: &str, f: Feature) {
        // TODO proper error for layer name not found
        let layer = self
            .layers
            .iter_mut()
            .find(|l| l.name == layer_name)
            .unwrap();
        layer.add_feature(f);
    }

    pub fn to_bytes(self) -> Vec<u8> {
        let mut res = Vec::new();
        self.write_to(&mut res);
        res
    }

    pub fn to_compressed_bytes(self) -> Vec<u8> {
        let mut compressor = GzEncoder::new(Vec::new(), Compression::none());

        // TODO this should return a Result, and we can then do something better than an assert
        self.write_to(&mut compressor);
        compressor.flush().unwrap();

        compressor.finish().unwrap()
    }

    pub fn write_to<W: Write>(self, writer: &mut W) {
        if self.is_empty() {
            return;
        }

        let converted: vector_tile::Tile = self.into();
        let mut cos = protobuf::CodedOutputStream::new(writer);
        converted.write_to(&mut cos).unwrap();
        cos.flush().unwrap();
    }

    pub fn write_to_file(self, filename: &str) {
        let mut file = BufWriter::new(File::create(filename).unwrap());
        file.write_all(&self.to_compressed_bytes()).unwrap();
    }

    pub fn is_empty(&self) -> bool {
        self.layers.iter().all(Layer::is_empty)
    }

    pub fn get_layer(&self, layer_name: impl AsRef<str>) -> Option<&Layer> {
        let layer_name: &str = layer_name.as_ref();
        self.layers.iter().find(|l| l.name == layer_name)
    }

    pub fn get_layer_mut(&mut self, layer_name: impl AsRef<str>) -> Option<&mut Layer> {
        let layer_name: &str = layer_name.as_ref();
        self.layers.iter_mut().find(|l| l.name == layer_name)
    }

    pub fn remove_layer(&mut self, layer_name: impl AsRef<str>) -> Option<Layer> {
        let layer_name: &str = layer_name.as_ref();
        let i = self
            .layers
            .iter()
            .enumerate()
            .filter_map(|(i, l)| if l.name == layer_name { Some(i) } else { None })
            .next();
        match i {
            Some(i) => Some(self.layers.remove(i)),
            None => None,
        }
    }
}

impl From<Tile> for vector_tile::Tile {
    fn from(v: Tile) -> Self {
        let mut result = Self::new();

        for layer in v.layers {
            if !layer.is_empty() {
                result.layers.push(layer.into());
            }
        }

        result
    }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum Value {
    String(Rc<String>),
    Float(f32),
    Double(f64),
    Int(i64),
    UInt(u64),
    SInt(i64),
    Boolean(bool),
    Unknown,
}

impl From<String> for Value {
    fn from(x: String) -> Self {
        Self::String(Rc::new(x))
    }
}
impl<'a> From<&'a str> for Value {
    fn from(x: &str) -> Self {
        Self::String(Rc::new(x.to_owned()))
    }
}

impl From<f32> for Value {
    fn from(x: f32) -> Self {
        Self::Float(x)
    }
}
impl From<f64> for Value {
    fn from(x: f64) -> Self {
        Self::Double(x)
    }
}
impl From<i64> for Value {
    fn from(x: i64) -> Self {
        Self::SInt(x)
    }
}
impl From<u64> for Value {
    fn from(x: u64) -> Self {
        Self::UInt(x)
    }
}
impl From<bool> for Value {
    fn from(x: bool) -> Self {
        Self::Boolean(x)
    }
}

impl From<vector_tile::tile::Value> for Value {
    fn from(val: vector_tile::tile::Value) -> Self {
        if val.has_string_value() {
            Self::String(Rc::new(val.string_value().into()))
        } else if val.has_float_value() {
            Self::Float(val.float_value())
        } else if val.has_double_value() {
            Self::Double(val.double_value())
        } else if val.has_int_value() {
            Self::Int(val.int_value())
        } else if val.has_uint_value() {
            Self::UInt(val.uint_value())
        } else if val.has_sint_value() {
            Self::SInt(val.sint_value())
        } else if val.has_bool_value() {
            Self::Boolean(val.bool_value())
        } else {
            Self::Unknown
        }
    }
}

impl From<Value> for vector_tile::tile::Value {
    fn from(val: Value) -> Self {
        let mut res = Self::new();
        match val {
            Value::String(s) => {
                res.set_string_value(Rc::try_unwrap(s).unwrap_or_else(|s| (*s).clone()))
            }
            Value::Float(s) => res.set_float_value(s),
            Value::Double(s) => res.set_double_value(s),
            Value::Int(s) => res.set_int_value(s),
            Value::UInt(s) => res.set_uint_value(s),
            Value::SInt(s) => res.set_sint_value(s),
            Value::Boolean(s) => res.set_bool_value(s),
            Value::Unknown => {}
        }

        res
    }
}

// FIXME use std::convert types, but it needs the mut
fn pbftile_to_tile(
    tile: vector_tile::Tile,
    invalid_geom_tactic: InvalidGeometryTactic,
) -> Result<Tile, Error> {
    Ok(Tile {
        layers: tile
            .layers
            .into_iter()
            .map(|l| pbflayer_to_layer(l, invalid_geom_tactic))
            .collect::<Result<Vec<_>, Error>>()?,
    })
}

fn pbflayer_to_layer(
    mut layer: vector_tile::tile::Layer,
    invalid_geom_tactic: InvalidGeometryTactic,
) -> Result<Layer, Error> {
    let name = layer.take_name();
    let extent = layer.extent();
    let features = layer.features;
    let keys = layer.keys;
    let values = layer.values;

    // TODO this is a filter in a ? so Some(Err(_)) is a bit messy
    let features: Vec<Feature> = features
        .into_iter()
        .filter_map(|f| {
            // TODO do we need the clone on values? I get a 'cannot move out of indexed context'
            // otherwise
            let properties: HashMap<Rc<String>, Value> = f
                .tags
                .chunks(2)
                .map(|kv: &[u32]| {
                    (
                        Rc::new(keys[kv[0] as usize].clone()),
                        values[kv[1] as usize].clone().into(),
                    )
                })
                .collect();

            match decode_geom(&f.geometry, &f.type_()) {
                Ok(geom) => Some(Ok(Feature {
                    properties: Rc::new(Properties(properties)),
                    geometry: geom,
                })),
                Err(e) => match invalid_geom_tactic {
                    InvalidGeometryTactic::StopProcessing => Some(Err(e)),
                    InvalidGeometryTactic::DropBrokenFeature => None,
                },
            }
        })
        .collect::<Result<Vec<_>, Error>>()?;

    Ok(Layer {
        name,
        features,
        extent,
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DrawingCommand {
    MoveTo(Vec<(i32, i32)>),
    LineTo(Vec<(i32, i32)>),
    ClosePath,
}

impl DrawingCommand {
    fn is_moveto(&self) -> bool {
        matches!(self, &Self::MoveTo(_))
    }

    // fn is_lineto(&self) -> bool {
    //     matches!(self, &DrawingCommand::LineTo(_))
    // }

    fn is_closepath(&self) -> bool {
        matches!(self, &Self::ClosePath)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrawingCommands(pub Vec<DrawingCommand>);

impl DrawingCommands {
    fn try_to_geometry(self) -> Result<Geometry<i32>, Error> {
        let commands = self;
        let geom_type = deduce_geom_type(&commands);

        match geom_type {
            vector_tile::tile::GeomType::POINT => {
                // Specs define this, but maybe we should make this return a Result instead of an
                // assertion
                assert_eq!(commands.0.len(), 1);
                match commands.0[0] {
                    DrawingCommand::MoveTo(ref points) => {
                        if points.is_empty() {
                            unreachable!()
                        } else if points.len() == 1 {
                            Ok(Geometry::Point(Point::new(points[0].0, points[0].1)))
                        } else if points.len() > 1 {
                            let mut cx = 0;
                            let mut cy = 0;
                            let mut new_points = Vec::with_capacity(points.len());
                            for p in points.iter() {
                                cx += p.0;
                                cy += p.1;
                                new_points.push(Point::new(cx, cy));
                            }
                            Ok(Geometry::MultiPoint(MultiPoint(new_points)))
                        } else {
                            unreachable!();
                        }
                    }
                    _ => {
                        unreachable!()
                    }
                }
            }
            vector_tile::tile::GeomType::LINESTRING => {
                let mut cx = 0;
                let mut cy = 0;
                assert_eq!(commands.0.len() % 2, 0);
                let mut lines: Vec<LineString<i32>> = Vec::with_capacity(commands.0.len() / 2);
                for cmds in commands.0.chunks(2) {
                    assert_eq!(cmds.len(), 2);

                    let mut linestring_points = Vec::new();

                    if let DrawingCommand::MoveTo(ref points) = cmds[0] {
                        assert_eq!(points.len(), 1);
                        let (dx, dy) = points[0];
                        cx += dx;
                        cy += dy;
                        linestring_points.push(Coord { x: cx, y: cy });
                    } else {
                        panic!("Invalid data");
                    }
                    if let DrawingCommand::LineTo(ref points) = cmds[1] {
                        assert!(!points.is_empty());
                        linestring_points.reserve(points.len());
                        for &(dx, dy) in points.iter() {
                            cx += dx;
                            cy += dy;
                            linestring_points.push(Coord { x: cx, y: cy });
                        }
                    } else {
                        panic!("Invalid data");
                    }

                    lines.push(LineString(linestring_points));
                }

                assert!(!lines.is_empty());
                if lines.len() == 1 {
                    Ok(Geometry::LineString(lines.remove(0)))
                } else {
                    Ok(Geometry::MultiLineString(MultiLineString(lines)))
                }
            }
            vector_tile::tile::GeomType::POLYGON => {
                let mut cx = 0;
                let mut cy = 0;
                if commands.0.len() % 3 != 0 {
                    return Err(InvalidDrawingCommands(commands));
                }
                let mut rings = Vec::with_capacity(commands.0.len() / 3);
                for cmds in commands.0.chunks(3) {
                    assert_eq!(cmds.len(), 3);
                    let mut linestring_points = Vec::new();

                    if let DrawingCommand::MoveTo(ref points) = cmds[0] {
                        assert_eq!(points.len(), 1);
                        let (dx, dy) = points[0];
                        cx += dx;
                        cy += dy;
                        linestring_points.push(Coord { x: cx, y: cy });
                    } else {
                        panic!("Invalid data");
                    }
                    if let DrawingCommand::LineTo(ref points) = cmds[1] {
                        if points.len() <= 1 {
                            // I have seen live data which is MoveTo([(77, -11)]), LineTo([(0,
                            // 0)]), ClosePath, that's invalid MVT data. "continue" means don't add
                            // linestring_points to rings, meaning this ring will be skipped
                            return Err(InvalidPolygonPointCount(points.len(), cmds.to_vec()));
                        }
                        linestring_points.reserve(points.len());
                        for &(dx, dy) in points.iter() {
                            cx += dx;
                            cy += dy;
                            linestring_points.push(Coord { x: cx, y: cy });
                        }
                    } else {
                        return Err(UnexpectedDrawingCommand(cmds.to_vec()));
                    }
                    if let DrawingCommand::ClosePath = cmds[2] {
                        // FIXME add first/last point
                        let line = LineString(linestring_points);
                        let winding_order = line.winding_order();
                        rings.push((line, winding_order));
                    } else {
                        panic!("Invalid data");
                    }
                }

                if rings.is_empty() {
                    return Err(NoValidRings);
                }
                let mut polygons = Vec::new();
                loop {
                    if rings.is_empty() {
                        break;
                    }
                    let (exterior_ring, _winding_order) = rings.remove(0);
                    let mut inner_rings = Vec::new();
                    if !rings.is_empty() {
                        loop {
                            if rings.is_empty() || rings[0].1 == Some(WindingOrder::Clockwise) {
                                break;
                            }
                            assert!(!rings.is_empty());
                            inner_rings.push(rings.remove(0).0);
                        }
                    }
                    polygons.push(Polygon::new(exterior_ring, inner_rings));
                }
                if polygons.is_empty() {
                    return Err(NoValidPolygons);
                }

                if polygons.len() == 1 {
                    // FIXME spec diff?
                    Ok(Geometry::Polygon(polygons.remove(0)))
                } else {
                    // if polygons.len() == 0 this will be triggered. probably not the best
                    Ok(Geometry::MultiPolygon(MultiPolygon(polygons)))
                }
            }
            vector_tile::tile::GeomType::UNKNOWN => unreachable!(),
        }
    }
}

impl<'a> From<&'a [u32]> for DrawingCommands {
    fn from(data: &[u32]) -> Self {
        let mut res = Vec::new();

        let mut idx = 0;
        loop {
            if idx >= data.len() {
                break;
            }

            let command = data[idx];
            idx += 1;
            let id = command & 0x7;
            let count = command >> 3;

            // Only MoveTo and LineTo take params
            let mut params = Vec::with_capacity(count as usize);
            if id == 1 || id == 2 {
                for _ in 0..count {
                    // XXX what about overflows here??
                    let value = data[idx] as i32;
                    idx += 1;
                    let d_x = (value >> 1) ^ (-(value & 1));

                    // XXX what about overflows here??
                    let value = data[idx] as i32;
                    idx += 1;
                    let d_y = (value >> 1) ^ (-(value & 1));

                    params.push((d_x, d_y));
                }
            }

            res.push(match id {
                1 => DrawingCommand::MoveTo(params),
                2 => DrawingCommand::LineTo(params),
                7 => DrawingCommand::ClosePath,
                _ => unreachable!(),
            });
        }

        Self(res)
    }
}

impl From<Vec<u32>> for DrawingCommands {
    fn from(data: Vec<u32>) -> Self {
        data.as_slice().into()
    }
}

fn decode_geom(
    data: &[u32],
    _geom_type: &vector_tile::tile::GeomType,
) -> Result<Geometry<i32>, Error> {
    let cmds: DrawingCommands = data.into();
    cmds.try_to_geometry()
}

fn deduce_geom_type(cmds: &DrawingCommands) -> vector_tile::tile::GeomType {
    let cmds = &cmds.0;

    // There are definitely ways for the input to be invalid where it should return UNKNOWN. e.g.
    // vec![DrawingCommands::ClosePath] would be POLYGON, but that's not a valid polygon, so it
    // should be UNKNOWN. But for valid DrawingCommands, it should be accurate.

    if cmds.len() == 1 && cmds[0].is_moveto() {
        vector_tile::tile::GeomType::POINT
    } else if cmds.iter().any(DrawingCommand::is_closepath) {
        vector_tile::tile::GeomType::POLYGON
    } else {
        vector_tile::tile::GeomType::LINESTRING
    }
}

fn move_cursor(current: &mut (i32, i32), point: &Coord<i32>) -> (i32, i32) {
    let x = point.x;
    let y = point.y;

    let dx = x - current.0;
    let dy = y - current.1;

    current.0 = x;
    current.1 = y;

    (dx, dy)
}

// FIXME these 2 conversions should be TryFrom which throw an error for non-integer coords
impl<'a> From<&'a Point<i32>> for DrawingCommands {
    fn from(point: &'a Point<i32>) -> Self {
        Self(vec![DrawingCommand::MoveTo(vec![(point.x(), point.y())])])
    }
}

impl<'a> From<&'a MultiPoint<i32>> for DrawingCommands {
    fn from(mp: &'a MultiPoint<i32>) -> Self {
        let mut offsets = Vec::with_capacity(mp.0.len());
        let mut cursor = (0, 0);
        for p in &mp.0 {
            offsets.push(move_cursor(&mut cursor, &p.0));
        }

        Self(vec![DrawingCommand::MoveTo(offsets)])
    }
}

impl<'a> From<&'a LineString<i32>> for DrawingCommands {
    fn from(ls: &'a LineString<i32>) -> Self {
        // FIXME error check <2 points
        let mut cmds = Vec::with_capacity(2);
        let mut cursor = (0, 0);
        cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
            &mut cursor,
            &ls.0[0],
        )]));

        let mut offsets = Vec::with_capacity(ls.0.len() - 1);
        for p in ls.0.iter().skip(1) {
            offsets.push(move_cursor(&mut cursor, p));
        }

        cmds.push(DrawingCommand::LineTo(offsets));

        Self(cmds)
    }
}

impl<'a> From<&'a MultiLineString<i32>> for DrawingCommands {
    fn from(mls: &'a MultiLineString<i32>) -> Self {
        // FIXME check for zero linestrings

        let mut cmds = Vec::with_capacity(2 * mls.0.len());
        let mut cursor = (0, 0);

        for (_line_idx, line) in mls.0.iter().enumerate() {
            cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
                &mut cursor,
                &line.0[0],
            )]));

            // FIXME error check <2 points

            let mut offsets = Vec::with_capacity(line.0.len() - 1);
            for p in line.0.iter().skip(1) {
                offsets.push(move_cursor(&mut cursor, p));
            }

            cmds.push(DrawingCommand::LineTo(offsets));
        }

        Self(cmds)
    }
}

impl<'a> From<&'a Polygon<i32>> for DrawingCommands {
    fn from(poly: &'a Polygon<i32>) -> Self {
        // Direction::Default means ext rings are ccw, but the vtile spec requires CW. *However*
        // vtiles have an inverted Y axis (Y is positive down), so this makes it work
        let poly = poly.orient(Direction::Default);

        let mut cmds = Vec::with_capacity(3 + 3 * poly.interiors().len());
        let mut cursor = (0, 0);

        cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
            &mut cursor,
            &poly.exterior().0[0],
        )]));

        let mut offsets = Vec::with_capacity(poly.exterior().0.len() - 1);
        for (i, p) in poly.exterior().0.iter().enumerate() {
            if i == 0 || i == poly.exterior().0.len() - 1 {
                continue;
            }
            offsets.push(move_cursor(&mut cursor, p));
        }

        cmds.push(DrawingCommand::LineTo(offsets));
        cmds.push(DrawingCommand::ClosePath);

        for int_ring in poly.interiors().iter() {
            cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
                &mut cursor,
                &int_ring.0[0],
            )]));

            let mut offsets = Vec::with_capacity(int_ring.0.len() - 1);
            for (i, p) in int_ring.0.iter().enumerate() {
                if i == 0 || i == int_ring.0.len() - 1 {
                    continue;
                }
                offsets.push(move_cursor(&mut cursor, p));
            }

            cmds.push(DrawingCommand::LineTo(offsets));
            cmds.push(DrawingCommand::ClosePath);
        }

        Self(cmds)
    }
}

impl<'a> From<&'a MultiPolygon<i32>> for DrawingCommands {
    fn from(mpoly: &'a MultiPolygon<i32>) -> Self {
        // Direction::Default means ext rings are ccw, but the vtile spec requires CW. *However*
        // vtiles have an inverted Y axis (Y is positive down), so this makes it work
        let mpoly = mpoly.orient(Direction::Default);
        let mut cmds = Vec::new();

        let mut cursor = (0, 0);

        for poly in mpoly.0 {
            cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
                &mut cursor,
                &poly.exterior().0[0],
            )]));

            let mut offsets = Vec::with_capacity(poly.exterior().0.len() - 1);
            for (i, p) in poly.exterior().0.iter().enumerate() {
                if i == 0 || i == poly.exterior().0.len() - 1 {
                    continue;
                }
                offsets.push(move_cursor(&mut cursor, p));
            }

            cmds.push(DrawingCommand::LineTo(offsets));
            cmds.push(DrawingCommand::ClosePath);

            for int_ring in poly.interiors().iter() {
                cmds.push(DrawingCommand::MoveTo(vec![move_cursor(
                    &mut cursor,
                    &int_ring.0[0],
                )]));

                let mut offsets = Vec::with_capacity(int_ring.0.len() - 1);
                for (i, p) in int_ring.0.iter().enumerate() {
                    if i == 0 || i == int_ring.0.len() - 1 {
                        continue;
                    }
                    offsets.push(move_cursor(&mut cursor, p));
                }

                cmds.push(DrawingCommand::LineTo(offsets));
                cmds.push(DrawingCommand::ClosePath);
            }
        }

        Self(cmds)
    }
}

impl<'a> From<&'a Geometry<i32>> for DrawingCommands {
    fn from(g: &'a Geometry<i32>) -> Self {
        match g {
            Geometry::Point(x) => x.into(),
            Geometry::LineString(x) => x.into(),
            Geometry::Polygon(x) => x.into(),
            Geometry::MultiPoint(x) => x.into(),
            Geometry::MultiLineString(x) => x.into(),
            Geometry::MultiPolygon(x) => x.into(),
            _ => unimplemented!(),
        }
    }
}

impl From<DrawingCommands> for Vec<u32> {
    fn from(dc: DrawingCommands) -> Self {
        let mut res = Self::with_capacity(dc.0.len());
        for cmd in dc.0 {
            match cmd {
                DrawingCommand::MoveTo(points) => {
                    res.reserve(1 + points.len() * 2);
                    let count = points.len() as u32;
                    let id: u32 = 1;
                    let cmd_int = (id & 0x7) | (count << 3);
                    res.push(cmd_int);
                    for (dx, dy) in points {
                        let dx = ((dx << 1) ^ (dx >> 31)) as u32;
                        let dy = ((dy << 1) ^ (dy >> 31)) as u32;
                        res.push(dx);
                        res.push(dy);
                    }
                }
                DrawingCommand::LineTo(points) => {
                    res.reserve(1 + points.len() * 2);
                    let count = points.len() as u32;
                    let id: u32 = 2;
                    let cmd_int = (id & 0x7) | (count << 3);
                    res.push(cmd_int);
                    for (dx, dy) in points {
                        let dx = ((dx << 1) ^ (dx >> 31)) as u32;
                        let dy = ((dy << 1) ^ (dy >> 31)) as u32;
                        res.push(dx);
                        res.push(dy);
                    }
                }
                DrawingCommand::ClosePath => {
                    //let count = 1;
                    //let id: u32 = 7
                    //let cmd_int = (id & 0x7) | (count << 3);
                    // id 7 and count of 1
                    res.push(15);
                }
            }
        }
        res
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_creation() {
        let mut t = Tile::new();
        let _l: Layer = Layer::new("fart".to_string());
        let _l: Layer = "hello".into();

        assert_eq!(t.layers.len(), 0);
        t.add_layer("poop");
        assert_eq!(t.layers.len(), 1);
        assert_eq!(t.layers[0].name, "poop");
        assert_eq!(t.layers[0].extent, 4096);

        t.add_feature(
            "poop",
            Feature::new(
                Geometry::Point(Point::new(10, 10)),
                Rc::new(Properties::new().set("name".to_owned(), "fart".to_owned())),
            ),
        );
    }

    #[test]
    fn test_properties() {
        let mut p = Properties::new();
        p.insert("name".to_string(), "bar");
        p.insert("visible".to_string(), false);

        let _p = Properties::new()
            .set("foo".to_string(), "bar".to_string())
            .set("num".to_string(), 10i64)
            .set("visible".to_string(), false);
    }

    #[test]
    fn encode_point() {
        let p = Point::new(25, 17);
        let dc: DrawingCommands = (&p).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![DrawingCommand::MoveTo(vec![(25, 17)])])
        );
        let bytes: Vec<u32> = dc.into();
        assert_eq!(bytes, vec![9, 50, 34]);
    }

    #[test]
    fn decode_point() {
        let bytes: Vec<u32> = vec![9, 50, 34];
        let dc: DrawingCommands = bytes.into();
        assert_eq!(
            dc,
            DrawingCommands(vec![DrawingCommand::MoveTo(vec![(25, 17)])])
        );
        let p: Geometry<i32> = dc.try_to_geometry().unwrap();
        assert_eq!(p, Geometry::Point(Point::new(25, 17)));
    }

    #[test]
    fn encode_linestring() {
        let ls: LineString<_> = vec![(2, 2), (2, 10), (10, 10)].into();
        let dc: DrawingCommands = (&ls).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![
                DrawingCommand::MoveTo(vec![(2, 2)]),
                DrawingCommand::LineTo(vec![(0, 8), (8, 0)])
            ])
        );
        let bytes: Vec<u32> = dc.into();
        assert_eq!(bytes, vec![9, 4, 4, 18, 0, 16, 16, 0]);
    }

    #[test]
    fn encode_polygon() {
        let ls1 = vec![(3, 6), (8, 12), (20, 34), (3, 6)].into();
        let poly = Polygon::new(ls1, vec![]);
        let dc: DrawingCommands = (&poly).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![
                DrawingCommand::MoveTo(vec![(3, 6)]),
                DrawingCommand::LineTo(vec![(5, 6), (12, 22)]),
                DrawingCommand::ClosePath
            ])
        );
        let bytes: Vec<u32> = dc.into();
        assert_eq!(bytes, vec![9, 6, 12, 18, 10, 12, 24, 44, 15]);
    }

    #[test]
    fn encode_polygon_with_hole() {
        let poly = Polygon::new(
            vec![(11, 11), (20, 11), (20, 20), (11, 20), (11, 11)].into(),
            vec![vec![(13, 13), (13, 17), (17, 17), (17, 13), (13, 13)].into()],
        );

        let dc: DrawingCommands = (&poly).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![
                DrawingCommand::MoveTo(vec![(11, 11)]),
                DrawingCommand::LineTo(vec![(9, 0), (0, 9), (-9, 0)]),
                DrawingCommand::ClosePath,
                DrawingCommand::MoveTo(vec![(2, -7)]),
                DrawingCommand::LineTo(vec![(0, 4), (4, 0), (0, -4)]),
                DrawingCommand::ClosePath,
            ])
        );

        let bytes: Vec<u32> = dc.into();
        assert_eq!(
            bytes,
            vec![9, 22, 22, 26, 18, 0, 0, 18, 17, 0, 15, 9, 4, 13, 26, 0, 8, 8, 0, 0, 7, 15]
        );
    }

    #[test]
    fn encode_multipoint() {
        let mp = MultiPoint(vec![Point::new(5, 7), Point::new(3, 2)]);
        let dc: DrawingCommands = (&mp).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![DrawingCommand::MoveTo(vec![(5, 7), (-2, -5)])])
        );
        let bytes: Vec<u32> = dc.into();
        assert_eq!(bytes, vec![17, 10, 14, 3, 9]);
    }

    #[test]
    fn decode_multipoint() {
        let bytes: Vec<u32> = vec![17, 10, 14, 3, 9];
        let dc: DrawingCommands = bytes.into();
        let mp: Geometry<i32> = dc.try_to_geometry().unwrap();
        assert_eq!(
            mp,
            Geometry::MultiPoint(MultiPoint(vec![Point::new(5, 7), Point::new(3, 2)]))
        );
    }

    #[test]
    fn encode_multilinestring() {
        let ls1 = vec![(2, 2), (2, 10), (10, 10)].into();
        let ls2 = vec![(1, 1), (3, 5)].into();
        let mls = MultiLineString(vec![ls1, ls2]);
        let dc: DrawingCommands = (&mls).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![
                DrawingCommand::MoveTo(vec![(2, 2)]),
                DrawingCommand::LineTo(vec![(0, 8), (8, 0)]),
                DrawingCommand::MoveTo(vec![(-9, -9)]),
                DrawingCommand::LineTo(vec![(2, 4)])
            ])
        );
        let bytes: Vec<u32> = dc.into();
        assert_eq!(bytes, vec![9, 4, 4, 18, 0, 16, 16, 0, 9, 17, 17, 10, 4, 8]);
    }

    #[test]
    fn encode_multipolygon() {
        let poly1 = Polygon::new(
            vec![(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)].into(),
            vec![],
        );
        let poly2 = Polygon::new(
            vec![(11, 11), (20, 11), (20, 20), (11, 20), (11, 11)].into(),
            vec![vec![(13, 13), (13, 17), (17, 17), (17, 13), (13, 13)].into()],
        );

        let mp = MultiPolygon(vec![poly1, poly2]);

        let dc: DrawingCommands = (&mp).into();
        assert_eq!(
            dc,
            DrawingCommands(vec![
                DrawingCommand::MoveTo(vec![(0, 0)]),
                DrawingCommand::LineTo(vec![(10, 0), (0, 10), (-10, 0)]),
                DrawingCommand::ClosePath,
                DrawingCommand::MoveTo(vec![(11, 1)]),
                DrawingCommand::LineTo(vec![(9, 0), (0, 9), (-9, 0)]),
                DrawingCommand::ClosePath,
                DrawingCommand::MoveTo(vec![(2, -7)]),
                DrawingCommand::LineTo(vec![(0, 4), (4, 0), (0, -4)]),
                DrawingCommand::ClosePath,
            ])
        );

        let bytes: Vec<u32> = dc.into();
        assert_eq!(
            bytes,
            vec![
                9, 0, 0, 26, 20, 0, 0, 20, 19, 0, 15, 9, 22, 2, 26, 18, 0, 0, 18, 17, 0, 15, 9, 4,
                13, 26, 0, 8, 8, 0, 0, 7, 15
            ]
        )
    }

    #[test]
    fn test_index() {
        let mut ti: TagIndex<String> = TagIndex::new();
        ti.add_item(&"baz".to_string());
        ti.add_item(&"foo".to_string());
        ti.add_item(&"bar".to_string());
        ti.add_item(&"bar".to_string());
        ti.add_item(&"bar".to_string());
        ti.add_item(&"baz".to_string());
        assert_eq!(ti.index_for_item(&"bar".to_string()), 0);
        assert_eq!(ti.index_for_item(&"baz".to_string()), 1);
        assert_eq!(ti.index_for_item(&"foo".to_string()), 2);
        //let keys: Vec<String> = ti.into_items().collect();
        //assert_eq!(keys, vec!["bar".to_string(), "baz".to_string(), "foo".to_string()]);
    }

    #[test]
    fn test_get_layers() {
        let mut t = Tile::new();
        let l1: Layer = Layer::new("fart".to_string());
        let l2: Layer = "hello".into();

        assert_eq!(t.layers.len(), 0);
        t.add_layer("poop");
        assert_eq!(t.layers.len(), 1);
        t.add_layer(l1);
        t.add_layer(l2);
        assert_eq!(t.layers.len(), 3);

        {
            let l1: &Layer = t.get_layer("fart").unwrap();
            assert_eq!(l1.name, "fart");
        }
        assert_eq!(t.layers.len(), 3);

        {
            let l: Layer = t.remove_layer("poop").unwrap();
            assert_eq!(l.name, "poop");
        }
        assert_eq!(t.layers.len(), 2);
    }
}
