//! Open surfaces: a sewn `TopAbs_SHELL` that is never upgraded to a solid.
//!
//! `Shell` is the second wrapper over the kind-generic carrier in
//! [`super::shape`] (that module records why a carrier was chosen over a copy
//! of `Solid`). It states one invariant — the native shape is a shell — and
//! borrows every kind-agnostic query from the carrier. `Solid`'s closed,
//! positive-volume guarantee is untouched: nothing here can produce a `Solid`,
//! and nothing in `Solid` can produce a `Shell`.
//!
//! Unlike `Solid`, the methods are inherent rather than a trait plus generated
//! delegations: there is one implementation, so a `ShellStruct` trait would add
//! a structure without adding a choice.

use super::edge::Edge;
use super::face::Face;
use super::ffi;
use super::shape::{Shape, ShapeKind};
use crate::common::error::Error;
use crate::common::mesh::Mesh;
use crate::traits::Tessellation;
use std::io::{Read, Write};

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
	fn code(self) -> u32 {
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
	fn code(self) -> u32 {
		match self {
			Continuity::C0 => 0,
			Continuity::G1 => 1,
			Continuity::G2 => 2,
		}
	}
}

/// Solver settings for [`Shell::fill`], one field per control of OCCT's
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

/// A single open or closed shell: a face set stitched into one `TopAbs_SHELL`.
///
/// A closed shell is representable here; it is simply not promoted to a solid.
/// Anything that needs the volume guarantee goes through `Solid` instead.
pub struct Shell(Shape);

impl Shell {
	/// Stitch free faces into one shell, merging boundary edges that coincide
	/// within `tolerance`. Unlike `Solid::sew` the result is kept as a shell
	/// whether or not it closed, but faces the sewing failed to attach are
	/// still rejected.
	pub fn sew<'a>(faces: impl IntoIterator<Item = &'a Face>, tolerance: f64) -> Result<Self, Error> {
		let mut face_vec = ffi::face_vec_new();
		for face in faces {
			ffi::face_vec_push(face_vec.pin_mut(), &face.inner);
		}
		let shape = ffi::make_sewn_shell(&face_vec, tolerance).map_err(|error| Error::Sew(error.to_string()))?;
		Self::from_native(shape, "sew")
	}

	/// Fill the region bounded by `boundary` with a single face, returned as a
	/// one-face shell. The edges are passed as constraints in the given order;
	/// several loops are filled by concatenating them.
	pub fn fill<'a>(boundary: impl IntoIterator<Item = &'a Edge>, filling: Filling) -> Result<Self, Error> {
		let mut edge_vec = ffi::edge_vec_new();
		for edge in boundary {
			ffi::edge_vec_push(edge_vec.pin_mut(), &edge.inner);
		}
		let shape = ffi::make_filled_shell(&edge_vec, filling.continuity.code(), filling.degree, filling.points_on_curve, filling.iterations, filling.max_degree, filling.max_segments, filling.tolerance_2d, filling.tolerance_3d, filling.tolerance_angular, filling.tolerance_curvature).map_err(|error| Error::Surface(error.to_string()))?;
		Self::from_native(shape, "fill")
	}

	/// Offset every face of the shell by signed `offset` along its normal.
	pub fn offset(&self, offset: f64, tolerance: f64, join: JoinType) -> Result<Self, Error> {
		let shape = ffi::make_offset_shell(self.0.inner(), offset, tolerance, join.code()).map_err(|error| Error::Offset(error.to_string()))?;
		Self::from_native(shape, "offset")
	}

	/// Boundary loops with only one adjacent face — empty for a closed shell.
	/// Loops are reported whole; a vertex where more than two boundary edges
	/// meet does not split them.
	pub fn free_boundaries(&self) -> Result<Vec<Vec<Edge>>, Error> {
		let mut loop_sizes: Vec<u32> = Vec::new();
		let edges = ffi::free_boundary_edges(self.0.inner(), false, false, &mut loop_sizes).map_err(|error| Error::Surface(error.to_string()))?;
		let mut taken = edges.iter();
		loop_sizes
			.into_iter()
			.map(|size| {
				(0..size)
					.map(|_| {
						let edge = taken.next().ok_or_else(|| Error::Surface("free boundaries: fewer edges than loop sizes (this is a bug)".into()))?;
						Edge::try_from_ffi(ffi::clone_edge_handle(edge), "free boundaries: null edge".into())
					})
					.collect()
			})
			.collect()
	}

	/// Topological kind of the underlying shape. Always [`ShapeKind::Shell`];
	/// the query exists so a decoded payload can be routed by kind.
	pub fn kind(&self) -> ShapeKind {
		self.0.kind()
	}

	/// `TopoDS_TShape*` of the shell, as `Solid::id` reports it for a solid.
	pub fn id(&self) -> u64 {
		self.0.id()
	}

	pub fn area(&self) -> f64 {
		self.0.area()
	}

	/// What OCCT's `BRepCheck_Analyzer` reports for the shell.
	pub fn is_valid(&self) -> Result<bool, Error> {
		self.0.is_valid()
	}

	pub fn iter_face(&self) -> impl Iterator<Item = &Face> + '_ {
		self.0.iter_face()
	}

	/// Triangulate shells for display. Per-triangle face ancestry is carried
	/// exactly as it is for solids.
	pub fn mesh<'a>(shells: impl IntoIterator<Item = &'a Shell>, options: Tessellation) -> Result<Mesh, Error> {
		super::io::mesh_shapes(shells.into_iter().map(|shell| shell.0.inner()), options)
	}

	/// Read every shell in a BRep payload. Unlike `Solid::read_brep` nothing is
	/// filtered out by solidity — but a solid in the payload does contribute
	/// its bounding shell, so a caller that stores both kinds tags the payload.
	pub fn read_brep<R: Read>(reader: &mut R) -> Result<Vec<Self>, Error> {
		Ok(super::io::read_brep_shapes(reader, ShapeKind::Shell)?.into_iter().map(Shell).collect())
	}

	pub fn write_brep<'a, W: Write>(shells: impl IntoIterator<Item = &'a Shell>, writer: &mut W) -> Result<(), Error> {
		super::io::write_brep_shapes(shells.into_iter().map(|shell| shell.0.inner()), writer)
	}

	fn from_native(shape: cxx::UniquePtr<ffi::TopoDS_Shape>, operation: &str) -> Result<Self, Error> {
		Shape::require(shape, ShapeKind::Shell, operation).map(Shell)
	}
}

impl std::fmt::Debug for Shell {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "Shell({}, faces={}, area={})", self.id(), self.iter_face().count(), self.area())
	}
}
