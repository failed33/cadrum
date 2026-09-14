//! Solids gathered into one compound and back, with their colours. The
//! compounding itself is [`Shape::compound`]; this is the colour bookkeeping
//! the I/O paths need around it.

use super::shape::{Shape, ShapeKind};
use super::solid::Solid;
#[cfg(feature = "color")]
use crate::common::color::Color;

pub(crate) struct CompoundShape {
	shape: Shape,
	#[cfg(feature = "color")]
	colormap: std::collections::HashMap<u64, Color>,
}

impl CompoundShape {
	/// Assemble solids into a compound, merging their colormaps. Their
	/// histories are not carried: a compound assembled for I/O has none.
	pub fn new<'a>(solids: impl IntoIterator<Item = &'a Solid>) -> Self {
		let solids: Vec<&Solid> = solids.into_iter().collect();
		#[cfg(feature = "color")]
		let colormap = solids.iter().flat_map(|solid| solid.colormap().iter().map(|(&key, &value)| (key, value))).collect();
		CompoundShape {
			shape: Shape::compound(solids.into_iter().map(Solid::as_shape)),
			#[cfg(feature = "color")]
			colormap,
		}
	}

	/// A shape read from a stream, with the colours read beside it.
	pub fn from_shape(shape: Shape, #[cfg(feature = "color")] colormap: std::collections::HashMap<u64, Color>) -> Self {
		CompoundShape {
			shape,
			#[cfg(feature = "color")]
			colormap,
		}
	}

	pub fn shape(&self) -> &Shape {
		&self.shape
	}

	#[cfg(feature = "color")]
	pub fn colormap(&self) -> &std::collections::HashMap<u64, Color> {
		&self.colormap
	}

	/// Every solid in the compound, each given the whole colormap.
	pub fn decompose(self) -> Vec<Solid> {
		self.shape
			.components(ShapeKind::Solid)
			.into_iter()
			.map(|solid| {
				Solid::new(
					solid,
					#[cfg(feature = "color")]
					self.colormap.clone(),
					Default::default(),
				)
			})
			.collect()
	}
}
