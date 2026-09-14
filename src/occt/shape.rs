//! Kind-generic native shape carrier.
//!
//! # Why a carrier and not a second `Solid`
//!
//! Wave 7a needed an open shell alongside the closed solid. Two designs were
//! weighed:
//!
//! 1. **A parallel `Shell` type duplicating `Solid`.** Every accessor, cache,
//!    compound assembly, BRep read and mesh path would exist twice, and "what a
//!    native shape carries" would become a decision recorded in two modules —
//!    the next kind would copy it a third time.
//! 2. **A kind-generic carrier (this module).** `Shape` holds any
//!    `TopoDS_Shape`, caches its faces, and answers [`Shape::kind`] instead of
//!    asserting one. The kind invariant belongs to the public wrapper that owns
//!    it: [`crate::Shell`] accepts only `TopAbs_SHELL`, and the decomposition,
//!    compound, BRep-read and meshing paths take a [`ShapeKind`] argument so
//!    solid and shell run the same code.
//!
//! Design 2 was taken. `Solid` keeps its own field layout and its closed,
//! positive-volume guarantee unchanged: rewriting an 836-line guarantee-bearing
//! type onto the carrier would be churn with no behavioural difference, and the
//! duplication worth removing — decomposition, compound assembly, BRep reading,
//! meshing — is already gone because those paths are kind-generic. What is still
//! written twice is the two-line face cache initialiser.

use super::face::Face;
use super::ffi;
use crate::common::error::Error;
use std::sync::OnceLock;

/// Topological kind of a native shape.
///
/// The codes are mirrored by `shape_kind` / `decompose_by_kind` in `ffi.cpp`
/// and are deliberately independent of OCCT's own enum ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeKind {
	Null,
	Compound,
	CompSolid,
	Solid,
	Shell,
	Face,
	Wire,
	Edge,
	Vertex,
	Other,
}

impl ShapeKind {
	pub(crate) fn code(self) -> u32 {
		match self {
			ShapeKind::Null => 0,
			ShapeKind::Compound => 1,
			ShapeKind::CompSolid => 2,
			ShapeKind::Solid => 3,
			ShapeKind::Shell => 4,
			ShapeKind::Face => 5,
			ShapeKind::Wire => 6,
			ShapeKind::Edge => 7,
			ShapeKind::Vertex => 8,
			ShapeKind::Other => 9,
		}
	}

	pub(crate) fn of(shape: &ffi::TopoDS_Shape) -> Self {
		match ffi::shape_kind(shape) {
			0 => ShapeKind::Null,
			1 => ShapeKind::Compound,
			2 => ShapeKind::CompSolid,
			3 => ShapeKind::Solid,
			4 => ShapeKind::Shell,
			5 => ShapeKind::Face,
			6 => ShapeKind::Wire,
			7 => ShapeKind::Edge,
			8 => ShapeKind::Vertex,
			_ => ShapeKind::Other,
		}
	}
}

/// A native shape of any kind, with its faces enumerated on first use.
///
/// The carrier imposes no invariant of its own — that is the wrapper's job —
/// so every query here is meaningful for a solid, a shell or a bare face.
pub(crate) struct Shape {
	inner: cxx::UniquePtr<ffi::TopoDS_Shape>,
	faces: OnceLock<Vec<Face>>,
}

impl Shape {
	pub(crate) fn new(inner: cxx::UniquePtr<ffi::TopoDS_Shape>) -> Self {
		Shape { inner, faces: OnceLock::new() }
	}

	/// Take `inner` only when it already has `kind`. `operation` names the
	/// producing call in the error, so a wrapper never has to phrase one.
	pub(crate) fn require(inner: cxx::UniquePtr<ffi::TopoDS_Shape>, kind: ShapeKind, operation: &str) -> Result<Self, Error> {
		let found = ShapeKind::of(&inner);
		if found != kind {
			return Err(Error::Validation(format!("{operation}: expected a {kind:?} shape, got {found:?}")));
		}
		Ok(Shape::new(inner))
	}

	pub(crate) fn kind(&self) -> ShapeKind {
		ShapeKind::of(&self.inner)
	}

	pub(crate) fn inner(&self) -> &ffi::TopoDS_Shape {
		&self.inner
	}

	pub(crate) fn into_inner(self) -> cxx::UniquePtr<ffi::TopoDS_Shape> {
		self.inner
	}

	pub(crate) fn id(&self) -> u64 {
		ffi::shape_tshape_id(&self.inner)
	}

	pub(crate) fn area(&self) -> f64 {
		ffi::shape_surface_area(&self.inner)
	}

	pub(crate) fn is_valid(&self) -> Result<bool, Error> {
		ffi::shape_is_valid(&self.inner).map_err(|error| Error::Validation(error.to_string()))
	}

	pub(crate) fn iter_face(&self) -> impl Iterator<Item = &Face> + '_ {
		self.faces.get_or_init(|| ffi::shape_faces(&self.inner).iter().map(|f| Face::new(ffi::clone_face_handle(f))).collect()).iter()
	}
}
