//! The algorithm table: one entry point over every OCCT algorithm the binding
//! offers.
//!
//! An [`Algorithm`] is one row: the inputs the OCCT builder takes and the
//! parameters it is constructed with, nothing else. [`apply`] runs it and
//! returns what every builder yields uniformly -- the shape, the lineage of
//! every input's faces, edges and vertices, and the faces the builder names
//! itself. Error translation and lineage extraction live once, in `ffi.cpp`,
//! behind `apply_algorithm`.
//!
//! # Designed twice
//!
//! 1. **An id, a slice of shapes and a parameter bag**: `apply(AlgorithmId,
//!    &[Shape], Params)`. The row's input arity and the meaning of each
//!    parameter would live in documentation and be checked at run time; a
//!    fillet's edges and a sweep's sections would both be "the shapes after
//!    the first one".
//! 2. **A registry of function pointers**, one per algorithm, registered at
//!    start-up. Open for extension without touching a match, but nothing
//!    fails to compile when a row is half added, and the parameters are
//!    untyped for the same reason as in (1).
//! 3. **One enum whose variants are the rows** (this module). Each variant
//!    names its inputs and parameters with their types, `apply` is one match
//!    that lays them out on the wire, and the compiler refuses a row that the
//!    match does not encode. The C++ side is the same switch over the same
//!    codes.
//!
//! (3) is taken. It is (1) with the inputs and parameters typed per row, and
//! it keeps the §8 rule of the operation API -- exhaustiveness at compile
//! time, no run-time registration.
//!
//! # The rows
//!
//! | Row | OCCT builder | Inputs | Parameters |
//! | --- | --- | --- | --- |
//! | `Box` | `BRepPrimAPI_MakeBox` | -- | two corners |
//! | `Sphere` | `BRepPrimAPI_MakeSphere` | -- | centre, radius |
//! | `Cylinder` | `BRepPrimAPI_MakeCylinder` | -- | base, axis, radius, height |
//! | `Cone` | `BRepPrimAPI_MakeCone` | -- | base, axis, two radii, height |
//! | `PeriodicGrid` | periodic tensor interpolation | corresponding grid samples | tolerance |
//! | `Torus` | `BRepPrimAPI_MakeTorus` | -- | centre, axis, two radii |
//! | `HalfSpace` | `BRepPrimAPI_MakeHalfSpace` | -- | plane origin, normal |
//! | `Wire` | `BRepBuilderAPI_MakeWire` | edges | -- |
//! | `Face` | `BRepBuilderAPI_MakeFace` | a wire | -- |
//! | `Prism` | `BRepPrimAPI_MakePrism` | a face or wire | vector |
//! | `Revolution` | `BRepPrimAPI_MakeRevol` | a face or wire | axis, angle |
//! | `PipeShell` | `BRepOffsetAPI_MakePipeShell` | spine wire, section wires | frame, scale law, tolerance, solid |
//! | `ThruSections` | `BRepOffsetAPI_ThruSections` | section wires | ruled, tolerance, solid |
//! | `OffsetShape` | `BRepOffsetAPI_MakeOffsetShape` | a shape | offset, tolerance, join, intersection |
//! | `OffsetFaces` | `BRepOffset_MakeOffset` | a shape, faces | offset, tolerance |
//! | `ThickSolid` | `BRepOffsetAPI_MakeThickSolid` | a solid, open faces | thickness, tolerance, join |
//! | `Solid` | `BRepBuilderAPI_MakeSolid` | a shell, cavity shells | -- |
//! | `Filling` | `BRepOffsetAPI_MakeFilling` | boundary edges | [`Filling`] |
//! | `Sew` | `BRepBuilderAPI_Sewing` | faces | tolerance |
//! | `Splitter` | `BRepAlgoAPI_Splitter` | arguments, tools | -- |
//! | `Section` | `BRepAlgoAPI_Section` | arguments, tools | -- |
//! | `Boolean` | `BOPAlgo_CellsBuilder` | shapes | a compact expression over them |
//! | `Fillet` | `BRepFilletAPI_MakeFillet` | a shape, edges | radius, radius law |
//! | `Chamfer` | `BRepFilletAPI_MakeChamfer` | a shape, edges, a reference face per edge | [`Bevel`] |
//! | `Transform` | `BRepBuilderAPI_GTransform` | a shape | a 3 by 4 affine matrix |
//! | `DraftAngle` | `BRepOffsetAPI_DraftAngle` | a shape, faces | direction, angle, neutral plane |
//! | `Projection` | `BRepProj_Projection` | a wire, a shape | direction |
//! | `Unify` | `ShapeUpgrade_UnifySameDomain` | a shape | -- |
//! | `Defeaturing` | `BRepAlgoAPI_Defeaturing` | a shape, faces | -- |
//!
//! `BRepFeat_*` is not a row: it lives in `TKFeat`, a toolkit the binding
//! does not link.

use super::edge::Edge;
use super::face::Face;
use super::ffi;
use super::shape::Shape;
use super::topology::{Descent, Landmark};
use crate::common::error::Error;
use boolean_expression::{Expression, Instruction, Operation};
use glam::DVec3;
use std::sync::{Mutex, PoisonError};

/// `BRepOffsetAPI_ThruSections` keeps global state and two concurrent lofts corrupt the heap. The lock
/// is held in Rust so ordinary algorithm errors release it when the call returns.
static LOFT: Mutex<()> = Mutex::new(());

/// Corner treatment where offset faces no longer meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinType {
	/// Round the corner with an arc of the offset radius.
	Arc,
	/// Extend the faces tangentially.
	Tangent,
	/// Extend the faces until they intersect, keeping the corner sharp.
	Intersection,
}

impl JoinType {
	fn code(self) -> i64 {
		match self {
			JoinType::Arc => 0,
			JoinType::Tangent => 1,
			JoinType::Intersection => 2,
		}
	}
}

/// Continuity required of a filling constraint.
///
/// `G1` and `G2` constrain the filled surface against the *faces* adjacent to
/// each boundary edge, so they are meaningful only for edges that have one; a
/// free wire supports `C0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Continuity {
	C0,
	G1,
	G2,
}

impl Continuity {
	fn code(self) -> i64 {
		match self {
			Continuity::C0 => 0,
			Continuity::G1 => 1,
			Continuity::G2 => 2,
		}
	}
}

/// Solver settings for [`Algorithm::Filling`], one field per control of OCCT's
/// `BRepOffsetAPI_MakeFilling`. `Default` reproduces OCCT's own documented
/// defaults, so a caller normally sets `continuity` and leaves the rest.
#[derive(Debug, Clone, Copy)]
pub struct Filling {
	/// Continuity required of every boundary constraint.
	pub continuity: Continuity,
	/// Starting degree of the filled surface.
	pub degree: u32,
	/// Points sampled on each boundary curve.
	pub points_on_curve: u32,
	/// Solver iterations before the result is accepted or refused.
	pub iterations: u32,
	/// Upper bound on the degree the solver may raise the surface to.
	pub max_degree: u32,
	/// Upper bound on the number of surface segments.
	pub max_segments: u32,
	/// Parametric-space tolerance.
	pub tolerance_2d: f64,
	/// Distance tolerance to the boundary constraints.
	pub tolerance_3d: f64,
	/// Angular tolerance, in radians, for `G1` constraints.
	pub tolerance_angular: f64,
	/// Relative curvature tolerance for `G2` constraints.
	pub tolerance_curvature: f64,
}

impl Default for Filling {
	fn default() -> Self {
		Filling {
			continuity: Continuity::C0,
			degree: 3,
			points_on_curve: 15,
			iterations: 2,
			max_degree: 8,
			max_segments: 9,
			tolerance_2d: 1.0e-5,
			tolerance_3d: 1.0e-4,
			tolerance_angular: 1.0e-2,
			tolerance_curvature: 1.0e-1,
		}
	}
}

/// How the section frame follows the spine of a [`Algorithm::PipeShell`].
#[derive(Debug, Clone, Copy)]
pub enum Frame {
	/// The frame at the spine's start, kept throughout.
	Fixed,
	/// The Frenet trihedron: the section twists with the curve's torsion.
	Frenet,
	/// The Frenet trihedron with the twist corrected out.
	CorrectedFrenet,
	/// The binormal held to this direction.
	Up(DVec3),
}

/// One station of a law sampled along a spine or a fillet contour: `value` at
/// normalised position `station`. A sweep reads it as the section's scale, a
/// fillet as the radius; the shape of the sample is the same either way, so
/// the rows share one type rather than restating it per law.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LawSample {
	pub station: f64,
	pub value: f64,
}

/// How a [`Algorithm::Chamfer`] is measured at each of its edges.
///
/// Both asymmetric forms measure their first parameter ON a reference face,
/// which is why the row takes one face per edge: OCCT needs to know which of
/// the two faces the edge separates the distance is laid out over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bevel {
	/// One distance, the same on both faces; no reference face is read.
	Symmetric { distance: f64 },
	/// `distance` on the reference face, `second` on the other one.
	Distances { distance: f64, second: f64 },
	/// `distance` on the reference face, the bevel rising from it at `angle`
	/// radians.
	Angle { distance: f64, angle: f64 },
}

impl Bevel {
	/// The row code and the one or two scalars the form carries.
	fn wire(self) -> (i64, [f64; 2], usize) {
		match self {
			Bevel::Symmetric { distance } => (0, [distance, 0.0], 1),
			Bevel::Distances { distance, second } => (1, [distance, second], 2),
			Bevel::Angle { distance, angle } => (2, [distance, angle], 2),
		}
	}
}

/// One row of the table: an OCCT algorithm with its inputs and parameters.
/// See the module documentation for the builder behind each row.
#[derive(Debug, Clone, Copy)]
pub enum Algorithm<'a> {
	/// Tensor-product interpolation, periodic in both directions, one sample per seam.
	PeriodicGrid {
		points: &'a [DVec3],
		rows: usize,
		columns: usize,
		tolerance: f64,
	},

	Box {
		corner: DVec3,
		opposite: DVec3,
	},
	Sphere {
		center: DVec3,
		radius: f64,
	},
	Cylinder {
		base: DVec3,
		axis: DVec3,
		radius: f64,
		height: f64,
	},
	Cone {
		base: DVec3,
		axis: DVec3,
		radius_at_base: f64,
		radius_at_top: f64,
		height: f64,
	},
	Torus {
		center: DVec3,
		axis: DVec3,
		major_radius: f64,
		minor_radius: f64,
	},
	/// The material lies on the side `normal` points to.
	HalfSpace {
		origin: DVec3,
		normal: DVec3,
	},
	Wire {
		edges: &'a [&'a Edge],
	},
	Face {
		wire: &'a Shape,
	},
	Prism {
		base: &'a Shape,
		vector: DVec3,
	},
	Revolution {
		base: &'a Shape,
		axis_origin: DVec3,
		axis_direction: DVec3,
		angle: f64,
	},
	/// Several sections morph from one to the next; a scale law takes exactly
	/// one. `tolerance` of `None` keeps the builder's own.
	PipeShell {
		spine: &'a Shape,
		sections: &'a [&'a Shape],
		frame: Frame,
		law: &'a [LawSample],
		tolerance: Option<f64>,
		solid: bool,
	},
	ThruSections {
		sections: &'a [&'a Shape],
		ruled: bool,
		tolerance: f64,
		solid: bool,
	},
	/// Every face offset by the same distance. `intersection` chooses OCCT's
	/// intersection algorithm over its join algorithm.
	OffsetShape {
		shape: &'a Shape,
		offset: f64,
		tolerance: f64,
		join: JoinType,
		intersection: bool,
	},
	/// Only `faces` offset; the rest of the shape stays where it is.
	OffsetFaces {
		shape: &'a Shape,
		faces: &'a [&'a Face],
		offset: f64,
		tolerance: f64,
	},
	/// A hollow solid whose wall is `thickness` thick, opened where
	/// `open_faces` were. At least one open face; a sealed hollow is an
	/// `OffsetShape` and a `Solid` with the offset shell as its cavity.
	ThickSolid {
		shape: &'a Shape,
		open_faces: &'a [&'a Face],
		thickness: f64,
		tolerance: f64,
		join: JoinType,
	},
	/// A solid bounded by `shell`, with `cavities` as internal voids. The
	/// result is oriented so its volume is positive.
	Solid {
		shell: &'a Shape,
		cavities: &'a [&'a Shape],
	},
	Filling {
		boundary: &'a [&'a Edge],
		filling: Filling,
	},
	/// The sewn shape as OCCT returns it: a shell, a lone face, or a compound
	/// of what did and did not attach.
	Sew {
		faces: &'a [&'a Face],
		tolerance: f64,
	},

	Splitter {
		arguments: &'a [&'a Shape],
		tools: &'a [&'a Shape],
	},
	Section {
		arguments: &'a [&'a Shape],
		tools: &'a [&'a Shape],
	},
	/// Evaluate a compact expression in one native splitting pass.
	Boolean {
		expression: &'a Expression<&'a Shape>,
	},

	/// `radius` at every station unless `law` is given, in which case the
	/// radius is interpolated through its stations along each contour and
	/// `radius` is unused.
	Fillet {
		shape: &'a Shape,
		edges: &'a [&'a Edge],
		radius: f64,
		law: &'a [LawSample],
	},
	/// `references` holds one face per edge, in the order `edges` states
	/// them, and is read only by the two asymmetric [`Bevel`] forms; the
	/// symmetric form takes none.
	Chamfer {
		shape: &'a Shape,
		edges: &'a [&'a Edge],
		references: &'a [&'a Face],
		bevel: Bevel,
	},
	/// A row-major 3 by 4 affine map. Topology is rebuilt; the history maps
	/// every face and edge onto its image.
	Transform {
		shape: &'a Shape,
		matrix: [f64; 12],
	},
	DraftAngle {
		shape: &'a Shape,
		faces: &'a [&'a Face],
		direction: DVec3,
		angle: f64,
		plane_origin: DVec3,
		plane_normal: DVec3,
	},
	/// `wire` projected along `direction` onto `onto`; a compound of wires.
	Projection {
		wire: &'a Shape,
		onto: &'a Shape,
		direction: DVec3,
	},
	/// Faces and edges on the same geometry merged.
	Unify {
		shape: &'a Shape,
	},
	/// `faces` removed from `shape` and the gap healed by extending the faces
	/// that adjoined them. The input is a solid, a compsolid or a compound of
	/// solids, as the builder requires.
	Defeaturing {
		shape: &'a Shape,
		faces: &'a [&'a Face],
	},
}

/// What [`apply`] returns.
#[derive(Debug)]
pub struct Applied {
	/// The built shape, exactly as the builder returned it.
	pub shape: Shape,
	/// Where each face, edge and vertex of the result came from, read through
	/// the builder's `IsDeleted` / `Modified` / `Generated` for every sub-shape
	/// of every input. An untouched input is [`Relation::Kept`]; a deleted one
	/// is absent.
	pub lineage: Vec<Descent>,
	/// The faces the builder names itself; see [`LandmarkRole`].
	pub landmarks: Vec<Landmark>,
}

impl Applied {
	/// The lineage as `[result, source]` `TShape` pairs, location-blind: what
	/// the [`crate::Solid`] colour map is keyed by.
	pub fn history(&self) -> Vec<[u64; 2]> {
		self.lineage.iter().map(|descent| [descent.result.tshape, descent.source.tshape]).collect()
	}
}

/// A shape taken as it is: no row ran, so it has no lineage and no landmarks.
impl From<Shape> for Applied {
	fn from(shape: Shape) -> Self {
		Applied { shape, lineage: Vec::new(), landmarks: Vec::new() }
	}
}

/// The wire form of one row: its code and the three argument slices
/// `apply_algorithm` reads by position.
struct Call {
	code: u32,
	shapes: cxx::UniquePtr<cxx::CxxVector<ffi::TopoDS_Shape>>,
	scalars: Vec<f64>,
	integers: Vec<i64>,
}

impl Call {
	fn new(code: u32) -> Self {
		Call { code, shapes: ffi::shape_vec_new(), scalars: Vec::new(), integers: Vec::new() }
	}
	fn shape(mut self, shape: &Shape) -> Self {
		ffi::shape_vec_push(self.shapes.pin_mut(), shape.inner());
		self
	}
	fn shapes<'a>(mut self, shapes: impl IntoIterator<Item = &'a &'a Shape>) -> Self {
		for shape in shapes {
			self = self.shape(shape);
		}
		self
	}
	fn edges<'a>(mut self, edges: impl IntoIterator<Item = &'a &'a Edge>) -> Self {
		for edge in edges {
			ffi::shape_vec_push_edge(self.shapes.pin_mut(), &edge.inner);
		}
		self
	}
	fn faces<'a>(mut self, faces: impl IntoIterator<Item = &'a &'a Face>) -> Self {
		for face in faces {
			ffi::shape_vec_push_face(self.shapes.pin_mut(), &face.inner);
		}
		self
	}
	fn vec(mut self, vector: DVec3) -> Self {
		self.scalars.extend(vector.to_array());
		self
	}
	fn scalar(mut self, value: f64) -> Self {
		self.scalars.push(value);
		self
	}
	fn scalars(mut self, values: impl IntoIterator<Item = f64>) -> Self {
		self.scalars.extend(values);
		self
	}
	fn integer(mut self, value: i64) -> Self {
		self.integers.push(value);
		self
	}
	fn count(self, value: usize) -> Self {
		self.integer(i64::try_from(value).unwrap_or(i64::MAX))
	}
}

impl Algorithm<'_> {
	/// The row's wire form. The layout here is what `ffi.cpp` reads back,
	/// case for case.
	fn call(self) -> Call {
		match self {
			Algorithm::PeriodicGrid { points, rows, columns, tolerance } => Call::new(29).count(rows).count(columns).scalar(tolerance).scalars(points.iter().flat_map(|point| point.to_array())),
			Algorithm::Box { corner, opposite } => Call::new(0).vec(corner.min(opposite)).vec(corner.max(opposite)),
			Algorithm::Sphere { center, radius } => Call::new(1).vec(center).scalar(radius),
			Algorithm::Cylinder { base, axis, radius, height } => Call::new(2).vec(base).vec(axis).scalar(radius).scalar(height),
			Algorithm::Cone { base, axis, radius_at_base, radius_at_top, height } => Call::new(3).vec(base).vec(axis).scalar(radius_at_base).scalar(radius_at_top).scalar(height),
			Algorithm::Torus { center, axis, major_radius, minor_radius } => Call::new(4).vec(center).vec(axis).scalar(major_radius).scalar(minor_radius),
			Algorithm::HalfSpace { origin, normal } => Call::new(5).vec(origin).vec(normal),
			Algorithm::Wire { edges } => Call::new(6).edges(edges),
			Algorithm::Face { wire } => Call::new(7).shape(wire),
			Algorithm::Prism { base, vector } => Call::new(8).shape(base).vec(vector),
			Algorithm::Revolution { base, axis_origin, axis_direction, angle } => Call::new(9).shape(base).vec(axis_origin).vec(axis_direction).scalar(angle),
			Algorithm::PipeShell { spine, sections, frame, law, tolerance, solid } => {
				let (code, up) = match frame {
					Frame::Fixed => (0, DVec3::ZERO),
					Frame::Frenet => (1, DVec3::ZERO),
					Frame::Up(direction) => (2, direction),
					Frame::CorrectedFrenet => (3, DVec3::ZERO),
				};
				Call::new(10).shape(spine).shapes(sections).integer(code).count(sections.len()).count(law.len()).integer(i64::from(solid)).vec(up).scalar(tolerance.unwrap_or(f64::NAN)).scalars(law.iter().map(|sample| sample.station)).scalars(law.iter().map(|sample| sample.value))
			}
			Algorithm::ThruSections { sections, ruled, tolerance, solid } => Call::new(11).shapes(sections).integer(i64::from(ruled)).integer(i64::from(solid)).scalar(tolerance),
			Algorithm::OffsetShape { shape, offset, tolerance, join, intersection } => Call::new(12).shape(shape).scalar(offset).scalar(tolerance).integer(join.code()).integer(i64::from(intersection)),
			Algorithm::OffsetFaces { shape, faces, offset, tolerance } => Call::new(13).shape(shape).faces(faces).scalar(offset).scalar(tolerance),
			Algorithm::ThickSolid { shape, open_faces, thickness, tolerance, join } => Call::new(14).shape(shape).faces(open_faces).scalar(thickness).scalar(tolerance).integer(join.code()),
			Algorithm::Solid { shell, cavities } => Call::new(15).shape(shell).shapes(cavities),
			Algorithm::Filling { boundary, filling } => Call::new(16).edges(boundary).integer(filling.continuity.code()).integer(i64::from(filling.degree)).integer(i64::from(filling.points_on_curve)).integer(i64::from(filling.iterations)).integer(i64::from(filling.max_degree)).integer(i64::from(filling.max_segments)).scalars([filling.tolerance_2d, filling.tolerance_3d, filling.tolerance_angular, filling.tolerance_curvature]),
			Algorithm::Sew { faces, tolerance } => Call::new(17).faces(faces).scalar(tolerance),

			Algorithm::Splitter { arguments, tools } => Call::new(19).shapes(arguments).shapes(tools).count(arguments.len()),
			Algorithm::Section { arguments, tools } => Call::new(20).shapes(arguments).shapes(tools).count(arguments.len()),
			Algorithm::Boolean { expression } => {
				let mut call = Call::new(21).shapes(expression.operands());
				call.integers.extend(expression.instructions().iter().map(|step| match *step {
					Instruction::Operand(index) => i64::try_from(index + 1).expect("operand index fits native address space"),
					Instruction::Combine(Operation::Union) => -1,
					Instruction::Combine(Operation::Difference) => -2,
					Instruction::Combine(Operation::Intersection) => -3,
				}));
				call
			}
			Algorithm::Fillet { shape, edges, radius, law } => Call::new(22).shape(shape).edges(edges).count(law.len()).scalar(radius).scalars(law.iter().map(|sample| sample.station)).scalars(law.iter().map(|sample| sample.value)),
			Algorithm::Chamfer { shape, edges, references, bevel } => {
				let (form, scalars, taken) = bevel.wire();
				Call::new(23).shape(shape).edges(edges).faces(references).integer(form).count(edges.len()).scalars(scalars.into_iter().take(taken))
			}
			Algorithm::Transform { shape, matrix } => Call::new(24).shape(shape).scalars(matrix),
			Algorithm::DraftAngle { shape, faces, direction, angle, plane_origin, plane_normal } => Call::new(25).shape(shape).faces(faces).vec(direction).scalar(angle).vec(plane_origin).vec(plane_normal),
			Algorithm::Projection { wire, onto, direction } => Call::new(26).shape(wire).shape(onto).vec(direction),
			Algorithm::Unify { shape } => Call::new(27).shape(shape),
			Algorithm::Defeaturing { shape, faces } => Call::new(28).shape(shape).faces(faces),
		}
	}
}

/// Run one row. Failure is [`Error::Algorithm`] carrying the row's name and
/// OCCT's reason; a null result never comes back.
pub fn apply(algorithm: Algorithm<'_>) -> Result<Applied, Error> {
	let _serialised = matches!(algorithm, Algorithm::ThruSections { .. }).then(|| LOFT.lock().unwrap_or_else(PoisonError::into_inner));
	let call = algorithm.call();
	let mut lineage = Vec::new();
	let mut landmarks = Vec::new();
	let mut refused = false;
	let shape = ffi::apply_algorithm(call.code, &call.shapes, &call.scalars, &call.integers, &mut lineage, &mut landmarks, &mut refused).map_err(|error| if refused { Error::Refused(error.to_string()) } else { Error::Algorithm(error.to_string()) })?;
	Ok(Applied { shape: Shape::new(shape), lineage: lineage.chunks_exact(5).filter_map(Descent::read).collect(), landmarks: landmarks.chunks_exact(3).filter_map(Landmark::read).collect() })
}
