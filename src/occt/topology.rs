//! The topology report: what a native shape is made of, read once.
//!
//! Every face, edge and vertex in `TopExp::MapShapes` order, each with its
//! located [`Key`], the definition of the geometry it lies on as OCCT
//! states it, and its incidence. The report is the enumeration the display
//! mesh, the lineage and every reference speak. The bridge carries it as
//! the typed rows of `ffi::TopologyData`; this module parses those once
//! into the facts below, so nothing past it reads a code, a sentinel or a
//! width.

use super::ffi;
use crate::common::error::Error;
use glam::DVec3;
use std::collections::HashMap;

/// Located identity of one sub-shape: the `TShape` and the hash of the
/// handle's location, the pair `TopoDS_Shape::IsSame` compares. A placed
/// copy has its own key. Valid while the shape that minted it lives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Key {
	pub tshape: u64,
	pub location: u64,
}

impl Key {
	fn read(words: &[u64]) -> Self {
		Key { tshape: words[0], location: words[1] }
	}

	fn parse(data: &ffi::KeyData) -> Self {
		Key { tshape: data.tshape, location: data.location }
	}
}

/// Where an analytic surface or a conic stands and how it is parametrised:
/// `gp_Ax3` less its derived y axis. `axis` is the surface's main direction
/// (a plane's normal, a cylinder's axis), `reference` the direction its
/// first parameter counts from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
	pub origin: DVec3,
	pub axis: DVec3,
	pub reference: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Axis {
	pub origin: DVec3,
	pub direction: DVec3,
}

/// What a spline surface is built from, short of its poles and knots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplineSurface {
	pub u_degree: u32,
	pub v_degree: u32,
	pub u_poles: u32,
	pub v_poles: u32,
	pub u_periodic: bool,
	pub v_periodic: bool,
	pub rational: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplineCurve {
	pub degree: u32,
	pub poles: u32,
	pub periodic: bool,
	pub rational: bool,
}

/// The surface a face lies on, by OCCT's `GeomAbs_SurfaceType`, with the
/// parameters that define it. Analytic kinds carry their whole definition;
/// a spline carries its shape, not its poles; `Other` is a surface OCCT
/// classifies no further.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceDefinition {
	Plane(Placement),
	Cylinder {
		placement: Placement,
		radius: f64,
	},
	/// `radius` at the placement's origin, widening at `semi_angle` radians
	/// along its axis.
	Cone {
		placement: Placement,
		radius: f64,
		semi_angle: f64,
	},
	Sphere {
		placement: Placement,
		radius: f64,
	},
	Torus {
		placement: Placement,
		major_radius: f64,
		minor_radius: f64,
	},
	Bezier(SplineSurface),
	BSpline(SplineSurface),
	Revolution(Axis),
	Extrusion(DVec3),
	Offset(f64),
	Other,
}

/// The curve an edge runs along, by OCCT's `GeomAbs_CurveType`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurveDefinition {
	Line(Axis),
	Circle { placement: Placement, radius: f64 },
	Ellipse { placement: Placement, major_radius: f64, minor_radius: f64 },
	Hyperbola { placement: Placement, major_radius: f64, minor_radius: f64 },
	Parabola { placement: Placement, focal: f64 },
	Bezier(SplineCurve),
	BSpline(SplineCurve),
	Offset(f64),
	Other,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceFact {
	pub key: Key,
	pub surface: SurfaceDefinition,
	/// Whether the face's outward side is the reverse of its surface's axis.
	pub reversed: bool,
	pub tolerance: f64,
	/// Positions of the edges bounding this face, in the report.
	pub edges: Vec<usize>,
}

impl FaceFact {
	/// The plane of a planar face: its origin and the face's OUTWARD normal.
	pub fn plane(&self) -> Option<(DVec3, DVec3)> {
		match self.surface {
			SurfaceDefinition::Plane(placement) => Some((placement.origin, if self.reversed { -placement.axis } else { placement.axis })),
			_ => None,
		}
	}
}

/// The curve of an edge that has one: its definition, its ends in the
/// forward parametrisation and its exact `GCPnts_AbscissaPoint` length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeGeometry {
	pub curve: CurveDefinition,
	pub start: DVec3,
	pub start_tangent: DVec3,
	pub end: DVec3,
	pub end_tangent: DVec3,
	pub length: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeFact {
	pub key: Key,
	/// `None` for a degenerate edge, which states no curve.
	pub geometry: Option<EdgeGeometry>,
	pub tolerance: f64,
	/// Positions of the start and end vertex; `None` where the edge has none.
	pub vertices: [Option<usize>; 2],
	/// Positions of the faces this edge bounds.
	pub faces: Vec<usize>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VertexFact {
	pub key: Key,
	pub point: DVec3,
	pub tolerance: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Topology {
	pub faces: Vec<FaceFact>,
	pub edges: Vec<EdgeFact>,
	pub vertices: Vec<VertexFact>,
	/// Axis-aligned extent from the geometry itself, independent of any
	/// triangulation the shape carries; `None` for a shape with nothing to
	/// measure.
	pub bounds: Option<[DVec3; 2]>,
	face_by_key: HashMap<Key, usize>,
	edge_by_key: HashMap<Key, usize>,
	vertex_by_key: HashMap<Key, usize>,
}

fn vec3(data: &ffi::Xyz) -> DVec3 {
	DVec3::new(data.x, data.y, data.z)
}

fn placement(data: &ffi::PlacementData) -> Placement {
	Placement { origin: vec3(&data.origin), axis: vec3(&data.axis), reference: vec3(&data.reference) }
}

fn axis(data: &ffi::AxisData) -> Axis {
	Axis { origin: vec3(&data.origin), direction: vec3(&data.direction) }
}

/// The value at `row` of a per-sub-shape column, or the report's own
/// inconsistency: a column shorter than the keys it goes with.
fn column<T>(values: &[T], row: usize) -> Result<&T, Error> {
	values.get(row).ok_or_else(|| Error::Validation("topology report column ends before its row".into()))
}

/// The definition a sub-shape's `(kind, row)` names.
fn definition<T>(values: &[T], row: u32) -> Result<&T, Error> {
	values.get(row as usize).ok_or_else(|| Error::Validation("topology report names no definition row".into()))
}

fn spline_surface(data: &ffi::SplineSurfaceDef) -> SplineSurface {
	SplineSurface { u_degree: data.u_degree, v_degree: data.v_degree, u_poles: data.u_poles, v_poles: data.v_poles, u_periodic: data.u_periodic, v_periodic: data.v_periodic, rational: data.rational }
}

fn spline_curve(data: &ffi::SplineCurveDef) -> SplineCurve {
	SplineCurve { degree: data.degree, poles: data.poles, periodic: data.periodic, rational: data.rational }
}

impl SurfaceDefinition {
	fn parse(data: &ffi::TopologyData, kind: ffi::SurfaceCode, row: u32) -> Result<Self, Error> {
		use ffi::SurfaceCode as Code;
		Ok(match kind {
			Code::Plane => Self::Plane(placement(&definition(&data.planes, row)?.placement)),
			Code::Cylinder => {
				let def = definition(&data.cylinders, row)?;
				Self::Cylinder { placement: placement(&def.placement), radius: def.radius }
			}
			Code::Cone => {
				let def = definition(&data.cones, row)?;
				Self::Cone { placement: placement(&def.placement), radius: def.radius, semi_angle: def.semi_angle }
			}
			Code::Sphere => {
				let def = definition(&data.spheres, row)?;
				Self::Sphere { placement: placement(&def.placement), radius: def.radius }
			}
			Code::Torus => {
				let def = definition(&data.tori, row)?;
				Self::Torus { placement: placement(&def.placement), major_radius: def.major_radius, minor_radius: def.minor_radius }
			}
			Code::Bezier => Self::Bezier(spline_surface(definition(&data.spline_surfaces, row)?)),
			Code::BSpline => Self::BSpline(spline_surface(definition(&data.spline_surfaces, row)?)),
			Code::Revolution => Self::Revolution(axis(&definition(&data.revolutions, row)?.axis)),
			Code::Extrusion => Self::Extrusion(vec3(&definition(&data.extrusions, row)?.direction)),
			Code::Offset => Self::Offset(definition(&data.surface_offsets, row)?.offset),
			Code::Other => Self::Other,
			_ => return Err(Error::Validation("topology report names an unknown surface kind".into())),
		})
	}
}

impl CurveDefinition {
	fn parse(data: &ffi::TopologyData, kind: ffi::CurveCode, row: u32) -> Result<Self, Error> {
		use ffi::CurveCode as Code;
		Ok(match kind {
			Code::Line => Self::Line(axis(&definition(&data.lines, row)?.axis)),
			Code::Circle => {
				let def = definition(&data.circles, row)?;
				Self::Circle { placement: placement(&def.placement), radius: def.radius }
			}
			Code::Ellipse => {
				let def = definition(&data.ellipses, row)?;
				Self::Ellipse { placement: placement(&def.placement), major_radius: def.major_radius, minor_radius: def.minor_radius }
			}
			Code::Hyperbola => {
				let def = definition(&data.hyperbolas, row)?;
				Self::Hyperbola { placement: placement(&def.placement), major_radius: def.major_radius, minor_radius: def.minor_radius }
			}
			Code::Parabola => {
				let def = definition(&data.parabolas, row)?;
				Self::Parabola { placement: placement(&def.placement), focal: def.focal }
			}
			Code::Bezier => Self::Bezier(spline_curve(definition(&data.spline_curves, row)?)),
			Code::BSpline => Self::BSpline(spline_curve(definition(&data.spline_curves, row)?)),
			Code::Offset => Self::Offset(definition(&data.curve_offsets, row)?.offset),
			Code::Other => Self::Other,
			_ => return Err(Error::Validation("topology report names an unknown curve kind".into())),
		})
	}
}

impl Topology {
	pub(crate) fn read(data: ffi::TopologyData) -> Result<Self, Error> {
		let csr = |offsets: &[u32], items: &[u32], row: usize, bound: usize| -> Result<Vec<usize>, Error> {
			let (start, end) = (*column(offsets, row)? as usize, *column(offsets, row + 1)? as usize);
			items.get(start..end).ok_or_else(|| Error::Validation("topology incidence out of range".into()))?.iter().map(|&item| (item as usize).lt(&bound).then_some(item as usize).ok_or_else(|| Error::Validation("topology incidence names no row".into()))).collect()
		};
		let (faces, edges) = (data.face_keys.len(), data.edge_keys.len());
		let face_facts = data
			.face_keys
			.iter()
			.enumerate()
			.map(|(face, key)| {
				Ok(FaceFact {
					key: Key::parse(key),
					surface: SurfaceDefinition::parse(&data, *column(&data.face_surface, face)?, *column(&data.face_definition, face)?)?,
					reversed: *column(&data.face_reversed, face)?,
					tolerance: *column(&data.face_tolerance, face)?,
					edges: csr(&data.face_edge_offsets, &data.face_edges, face, edges)?,
				})
			})
			.collect::<Result<Vec<_>, Error>>()?;
		let edge_facts = data
			.edge_keys
			.iter()
			.enumerate()
			.map(|(edge, key)| {
				let kind = *column(&data.edge_curve, edge)?;
				let geometry = (kind != ffi::CurveCode::Degenerate)
					.then(|| {
						let ends = column(&data.edge_ends, edge)?;
						Ok::<_, Error>(EdgeGeometry {
							curve: CurveDefinition::parse(&data, kind, *column(&data.edge_definition, edge)?)?,
							start: vec3(&ends.start),
							start_tangent: vec3(&ends.start_tangent),
							end: vec3(&ends.end),
							end_tangent: vec3(&ends.end_tangent),
							length: *column(&data.edge_length, edge)?,
						})
					})
					.transpose()?;
				let vertex = |slot: usize| -> Result<Option<usize>, Error> {
					let index = *column(&data.edge_vertices, edge * 2 + slot)?;
					Ok((index != u32::MAX).then_some(index as usize))
				};
				Ok(EdgeFact {
					key: Key::parse(key),
					geometry,
					tolerance: *column(&data.edge_tolerance, edge)?,
					vertices: [vertex(0)?, vertex(1)?],
					faces: csr(&data.edge_face_offsets, &data.edge_faces, edge, faces)?,
				})
			})
			.collect::<Result<Vec<_>, Error>>()?;
		let vertex_facts = data.vertex_keys.iter().enumerate().map(|(vertex, key)| Ok(VertexFact { key: Key::parse(key), point: vec3(column(&data.vertex_points, vertex)?), tolerance: *column(&data.vertex_tolerance, vertex)? })).collect::<Result<Vec<_>, Error>>()?;
		Ok(Topology {
			face_by_key: face_facts.iter().enumerate().map(|(index, fact)| (fact.key, index)).collect(),
			edge_by_key: edge_facts.iter().enumerate().map(|(index, fact)| (fact.key, index)).collect(),
			vertex_by_key: vertex_facts.iter().enumerate().map(|(index, fact)| (fact.key, index)).collect(),
			faces: face_facts,
			edges: edge_facts,
			vertices: vertex_facts,
			bounds: data.bounded.then(|| [vec3(&data.extent.low), vec3(&data.extent.high)]),
		})
	}

	pub fn face_of(&self, key: Key) -> Option<usize> {
		self.face_by_key.get(&key).copied()
	}

	pub fn edge_of(&self, key: Key) -> Option<usize> {
		self.edge_by_key.get(&key).copied()
	}

	pub fn vertex_of(&self, key: Key) -> Option<usize> {
		self.vertex_by_key.get(&key).copied()
	}
}

/// How a result sub-shape relates to the input sub-shape it descends from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Relation {
	/// The input stands in the result as it was.
	Kept,
	/// The builder rebuilt the input into the result element.
	Modified,
	/// The builder made the result element from the input (a swept edge's
	/// lateral face, a swept vertex's edge).
	Generated,
}

/// One hop of lineage: a result sub-shape and the input sub-shape it came
/// from. Keys of different kinds meet here (an edge generates a face).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Descent {
	pub relation: Relation,
	pub result: Key,
	pub source: Key,
}

impl Descent {
	pub(crate) fn read(words: &[u64]) -> Option<Self> {
		let relation = match words[0] {
			0 => Relation::Kept,
			1 => Relation::Modified,
			2 => Relation::Generated,
			_ => return None,
		};
		Some(Descent { relation, result: Key::read(&words[1..3]), source: Key::read(&words[3..5]) })
	}
}

/// A face a builder names itself: a box side by the axis it faces, a
/// one-axis primitive's bottom, top and lateral face, or the first and last
/// section of a prism, revolution, pipe or loft.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LandmarkRole {
	XMin,
	XMax,
	YMin,
	YMax,
	ZMin,
	ZMax,
	Bottom,
	Top,
	Lateral,
	First,
	Last,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Landmark {
	pub role: LandmarkRole,
	pub face: Key,
}

impl Landmark {
	pub(crate) fn read(words: &[u64]) -> Option<Self> {
		use LandmarkRole::*;
		let role = *[XMin, XMax, YMin, YMax, ZMin, ZMax, Bottom, Top, Lateral, First, Last].get(words[0] as usize)?;
		Some(Landmark { role, face: Key::read(&words[1..3]) })
	}
}

/// The sub-shape a probe point is closest to, and where on it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nearest {
	pub support: Support,
	pub point: DVec3,
	/// The outward normal where the support is a face and OCCT can define one.
	pub normal: Option<DVec3>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Support {
	Vertex(Key),
	Edge(Key),
	Face(Key),
}

impl Nearest {
	pub(crate) fn read(data: ffi::NearestData) -> Option<Self> {
		let key = Key { tshape: data.tshape, location: data.location };
		let support = match data.support {
			1 => Support::Vertex(key),
			2 => Support::Edge(key),
			3 => Support::Face(key),
			_ => return None,
		};
		let normal = DVec3::new(data.nx, data.ny, data.nz);
		Some(Nearest { support, point: DVec3::new(data.px, data.py, data.pz), normal: normal.is_finite().then_some(normal) })
	}
}
