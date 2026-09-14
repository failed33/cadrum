use super::ffi;
use super::shape::{Shape, ShapeKind};
use super::solid::Solid;
#[cfg(feature = "color")]
use crate::common::color::Color;

/// A compound shape wrapping multiple solids into a single `TopoDS_Compound`.
///
/// Provides type-safe distinction from individual `Solid` handles.
pub(crate) struct CompoundShape {
	inner: cxx::UniquePtr<ffi::TopoDS_Shape>,
	#[cfg(feature = "color")]
	colormap: std::collections::HashMap<u64, Color>,
	history: Vec<u64>,
}

impl CompoundShape {
	/// Assemble solids into a compound, merging their colormaps.
	///
	/// Inputs' `history` is intentionally dropped — a compound assembled for
	/// a boolean call has no meaningful history of its own; the boolean
	/// result will populate one fresh.
	pub fn new<'a>(solids: impl IntoIterator<Item = &'a Solid>) -> Self {
		#[cfg(feature = "color")]
		let mut colormap = std::collections::HashMap::new();
		let solids: Vec<&Solid> = solids.into_iter().collect();
		#[cfg(feature = "color")]
		for s in &solids {
			colormap.extend(s.colormap().iter().map(|(&k, &v)| (k, v)));
		}
		let inner = compound_of(solids.into_iter().map(Solid::inner));
		CompoundShape {
			inner,
			#[cfg(feature = "color")]
			colormap,
			history: Default::default(),
		}
	}

	/// Create a compound from a raw `TopoDS_Shape` (e.g. from I/O or boolean ops).
	pub fn from_raw(inner: cxx::UniquePtr<ffi::TopoDS_Shape>, #[cfg(feature = "color")] colormap: std::collections::HashMap<u64, Color>, history: Vec<u64>) -> Self {
		CompoundShape {
			inner,
			#[cfg(feature = "color")]
			colormap,
			history,
		}
	}

	/// Borrow the underlying `TopoDS_Shape`.
	pub fn inner(&self) -> &ffi::TopoDS_Shape {
		&self.inner
	}

	/// Borrow the merged colormap.
	#[cfg(feature = "color")]
	pub fn colormap(&self) -> &std::collections::HashMap<u64, Color> {
		&self.colormap
	}

	/// Decompose into individual solids, consuming the compound.
	///
	/// Each result solid receives a clone of the full `history` — over-inclusion
	/// is harmless because `iter_history()` consumers filter pairs by checking
	/// `src_id` against the original input's face IDs.
	pub fn decompose(self) -> Vec<Solid> {
		shapes_of_kind(&self.inner, ShapeKind::Solid)
			.into_iter()
			.map(|shape| {
				Solid::new(
					shape.into_inner(),
					#[cfg(feature = "color")]
					self.colormap.clone(),
					self.history.clone(),
				)
			})
			.collect()
	}
}

/// Assemble any shapes into one `TopoDS_Compound`. The kind-generic half of
/// [`CompoundShape::new`]: shells and solids are gathered identically, only
/// the colormap merge above is solid-specific.
pub(crate) fn compound_of<'a>(shapes: impl IntoIterator<Item = &'a ffi::TopoDS_Shape>) -> cxx::UniquePtr<ffi::TopoDS_Shape> {
	let mut inner = ffi::make_empty();
	for shape in shapes {
		ffi::compound_add(inner.pin_mut(), shape);
	}
	inner
}

/// Every sub-shape of `kind`, as carriers. Owns the handle clone so callers
/// never hold a borrow of the compound they decomposed.
pub(crate) fn shapes_of_kind(shape: &ffi::TopoDS_Shape, kind: ShapeKind) -> Vec<Shape> {
	ffi::decompose_by_kind(shape, kind.code()).iter().map(|found| Shape::new(ffi::clone_shape_handle(found))).collect()
}
