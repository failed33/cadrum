//! Solid I/O: the colour trailer and the colormap around the kind-generic
//! payload, triangulation and STEP paths of [`Shape`]. Exposed via
//! `impl SolidStruct for Solid` (`Solid::read_step`, `Solid::mesh`, ...).

use super::compound::CompoundShape;
use super::shape::Shape;
#[cfg(feature = "color")]
use super::shape::ShapeKind;
use super::solid::Solid;
use crate::common::error::Error;
use crate::common::mesh::Mesh;
use crate::traits::Tessellation;
use std::io::{Read, Write};

#[cfg(feature = "color")]
use crate::common::color::Color;

// ==================== Color trailer ====================
// Appended past the BinTools payload, which BinTools::Read stops at and ignores:
// `[b"CDCL"][u32 count][count x (u32 trailer_ids index, f32 r, f32 g, f32 b)]`, LE.

#[cfg(feature = "color")]
const COLOR_TRAILER_MAGIC: &[u8; 4] = b"CDCL";

/// `tail` is `&buf[consumed..]`, the bytes the BRep parser did not take. Anything that
/// is not our trailer yields an empty map — the geometry is valid either way.
#[cfg(feature = "color")]
fn read_color_trailer(tail: &[u8]) -> std::collections::HashMap<u32, Color> {
	let mut colormap = std::collections::HashMap::new();
	if tail.len() < 8 || &tail[..4] != COLOR_TRAILER_MAGIC {
		return colormap;
	}
	let count = u32::from_le_bytes(tail[4..8].try_into().unwrap()) as usize;
	// `count` comes from the file, and `usize` is 32-bit on wasm32.
	let Some(end) = count.checked_mul(16).and_then(|n| n.checked_add(8)) else {
		return colormap;
	};
	// `<`, not `!=`: the count self-delimits, so bytes appended after us are not an error.
	if tail.len() < end {
		return colormap;
	}
	for e in tail[8..end].chunks_exact(16) {
		let idx = u32::from_le_bytes(e[0..4].try_into().unwrap());
		let r = f32::from_le_bytes(e[4..8].try_into().unwrap());
		let g = f32::from_le_bytes(e[8..12].try_into().unwrap());
		let b = f32::from_le_bytes(e[12..16].try_into().unwrap());
		colormap.insert(idx, Color { r, g, b });
	}
	colormap
}

/// STEP cannot index like this — `try_sew_orphan_faces` shifts every index, so it
/// carries explicit ids instead.
#[cfg(feature = "color")]
fn trailer_ids(shape: &Shape) -> Vec<u64> {
	// Bound to locals: both are `UniquePtr<CxxVector<..>>` that the iterators borrow.
	shape.components(ShapeKind::Solid).iter().map(Shape::id).chain(shape.iter_face().map(|face| face.id())).collect()
}

#[cfg(feature = "color")]
fn write_color_trailer<W: Write>(compound: &CompoundShape, writer: &mut W) -> Result<(), Error> {
	let id_to_index: std::collections::HashMap<u64, u32> = trailer_ids(compound.shape()).into_iter().enumerate().map(|(i, id)| (id, i as u32)).collect();
	// `CompoundShape::decompose` gives every solid a clone of the merged colormap, so
	// a solid carries its siblings' keys too; those have no index and drop out here.
	let mut entries: Vec<(u32, f32, f32, f32)> = compound.colormap().iter().filter_map(|(id, rgb)| id_to_index.get(id).map(|&idx| (idx, rgb.r, rgb.g, rgb.b))).collect();
	if entries.is_empty() {
		return Ok(());
	}
	entries.sort_by_key(|e| e.0);

	let mut out = Vec::with_capacity(8 + entries.len() * 16);
	out.extend_from_slice(COLOR_TRAILER_MAGIC);
	out.extend_from_slice(&(entries.len() as u32).to_le_bytes());
	for (idx, r, g, b) in &entries {
		out.extend_from_slice(&idx.to_le_bytes());
		out.extend_from_slice(&r.to_le_bytes());
		out.extend_from_slice(&g.to_le_bytes());
		out.extend_from_slice(&b.to_le_bytes());
	}
	writer.write_all(&out).map_err(Error::Io)
}

// ==================== Reader / writer / mesh helpers ====================
//
// Each function is invoked by the matching `SolidStruct` method in
// `super::solid::Solid`. Kept module-private (`pub(super)`) so the public
// surface lives entirely on `Solid`.

pub(super) fn read_step<R: Read>(reader: &mut R) -> Result<Vec<Solid>, Error> {
	let shape = Shape::read_step(reader)?;
	#[cfg(feature = "color")]
	let compound = CompoundShape::from_shape(shape, Default::default());
	#[cfg(not(feature = "color"))]
	let compound = CompoundShape::from_shape(shape);
	Ok(compound.decompose())
}

pub(super) fn read_brep<R: Read>(reader: &mut R) -> Result<Vec<Solid>, Error> {
	#[cfg_attr(not(feature = "color"), allow(unused_variables))]
	let (shape, buf, consumed) = Shape::read_brep_payload(reader)?;

	#[cfg(feature = "color")]
	{
		let ids = trailer_ids(&shape);
		let colormap = read_color_trailer(buf.get(consumed..).unwrap_or_default()).into_iter().filter_map(|(idx, color)| ids.get(idx as usize).map(|&id| (id, color))).collect();
		Ok(CompoundShape::from_shape(shape, colormap).decompose())
	}
	#[cfg(not(feature = "color"))]
	{
		Ok(CompoundShape::from_shape(shape).decompose())
	}
}

/// Write solids to a STEP stream. Geometry only: colours travel in the BRep
/// trailer, never in STEP.
pub(super) fn write_step<'a, W: Write>(solids: impl IntoIterator<Item = &'a Solid>, writer: &mut W) -> Result<(), Error> {
	Shape::write_step([CompoundShape::new(solids).shape()], writer)
}

pub(super) fn write_brep<'a, W: Write>(solids: impl IntoIterator<Item = &'a Solid>, writer: &mut W) -> Result<(), Error> {
	let compound = CompoundShape::new(solids);
	// The payload lands before the trailer: the streambuf flushes on drop.
	Shape::write_brep([compound.shape()], writer)?;
	#[cfg(feature = "color")]
	write_color_trailer(&compound, writer)?;
	Ok(())
}

pub(super) fn mesh<'a>(solids: impl IntoIterator<Item = &'a Solid>, options: Tessellation) -> Result<Mesh, Error> {
	let solids: Vec<&Solid> = solids.into_iter().collect();
	// `Mesh` has only a face level, so a solid-level colour is expanded onto its faces
	// here. STEP and the BRep trailer keep the distinction; the renderers cannot.
	#[cfg(feature = "color")]
	let face_colors = {
		let mut map = std::collections::HashMap::new();
		for s in &solids {
			if let Some(&c) = s.colormap().get(&s.id()) {
				for f in s.iter_face() {
					map.insert(f.id(), c);
				}
			}
			// Face colours are the more specific style and win over the solid's.
			map.extend(s.colormap().iter().map(|(&k, &v)| (k, v)));
		}
		map
	};

	let mesh = Shape::mesh(solids.iter().map(|solid| solid.as_shape()), options)?;

	#[cfg(feature = "color")]
	let mesh = {
		let mut mesh = mesh;
		mesh.colormap = mesh.face_ids.iter().filter_map(|id| face_colors.get(id).map(|&color| (*id, color))).collect();
		mesh
	};
	Ok(mesh)
}
