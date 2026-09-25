//! The kind-generic native shape carrier.
//!
//! `Shape` holds any `TopoDS_Shape` and answers [`Shape::kind`] instead of
//! asserting one; every query here is meaningful for a solid, a shell, a face
//! or a wire. Operations are not methods of the carrier: they are rows of the
//! algorithm table in [`super::algorithm`], which takes carriers in and hands
//! one back. A caller that needs a kind guarantee -- [`crate::Solid`] does --
//! checks the kind of what the table returned; the carrier never does.
//!
//! # Designed twice
//!
//! 1. **One wrapper type per topological kind** (`Solid`, `Shell`, ...), each
//!    with its own cache, compound assembly, BRep read and mesh path. Wave 7a
//!    added `Shell` beside `Solid` this way and stopped at "what a native shape
//!    carries is written in two modules"; the next kind copies it a third time.
//! 2. **One carrier whose kind is data** (this module). Enumeration, caches,
//!    compounding, decomposition, I/O and triangulation are written once;
//!    `Solid` is a thin guarantee over it.
//!
//! (2) is taken, and the 7a decision to leave `Solid`'s own layout in place
//! is reversed here: `Solid` now holds a `Shape`, because the duplication the
//! ruling of 2026-09-14 counts is exactly the per-kind copy.

use super::edge::Edge;
use super::face::Face;
use super::ffi;
use super::ffi::RustReader;
use super::ffi::RustWriter;
use super::topology::{Key, Nearest, Topology};
use crate::common::error::Error;
use crate::common::mesh::Mesh;
use crate::traits::Tessellation;
use glam::{DMat3, DVec3};
use std::io::{Read, Write};
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

/// What [`Shape::planar_region`] answers: the bounded face, its plane, and
/// which face edge copies which wire edge.
#[derive(Debug)]
pub struct PlanarRegion {
	pub face: Shape,
	pub origin: DVec3,
	pub normal: DVec3,
	pub edges: Vec<(Key, Key)>,
}

/// A native shape of any kind, with its edges and faces enumerated on first
/// use. The caches belong to this handle: a shape returned by the table or
/// by a placement is a fresh carrier with fresh caches.
/// Native topology may be shared by shallow clones, so it stays on one thread.
/// ```compile_fail
/// fn require_send<T: Send>() {}
/// require_send::<cadrum::Shape>();
/// ```
pub struct Shape {
	inner: cxx::UniquePtr<ffi::TopoDS_Shape>,
	edges: OnceLock<Vec<Edge>>,
	faces: OnceLock<Vec<Face>>,
	topology: OnceLock<Topology>,
}

impl Shape {
	pub(crate) fn new(inner: cxx::UniquePtr<ffi::TopoDS_Shape>) -> Self {
		Shape { inner, edges: OnceLock::new(), faces: OnceLock::new(), topology: OnceLock::new() }
	}

	/// What this shape is made of, read once: see [`Topology`]. The face and
	/// edge positions it states are the positions of [`Shape::iter_face`] and
	/// [`Shape::iter_edge`], and the ones a display mesh names.
	pub fn topology(&self) -> Result<&Topology, Error> {
		if let Some(report) = self.topology.get() {
			return Ok(report);
		}
		let report = Topology::read(ffi::shape_topology(&self.inner).map_err(|error| Error::Validation(error.to_string()))?)?;
		Ok(self.topology.get_or_init(|| report))
	}

	/// Located identity; see [`Key`].
	pub fn key(&self) -> Key {
		let (mut tshape, mut location) = (0, 0);
		ffi::shape_key(&self.inner, &mut tshape, &mut location);
		Key { tshape, location }
	}

	/// The sub-shape closest to `point` and where on it; `None` for a shape
	/// with nothing to measure against.
	pub fn nearest(&self, point: DVec3) -> Result<Option<Nearest>, Error> {
		ffi::shape_nearest(&self.inner, point.x, point.y, point.z).map(Nearest::read).map_err(|error| Error::Properties(error.to_string()))
	}

	pub(crate) fn inner(&self) -> &ffi::TopoDS_Shape {
		&self.inner
	}

	/// One compound holding every shape, in order. The handles are shared,
	/// so every id stays what the caller holds.
	pub fn compound<'a>(shapes: impl IntoIterator<Item = &'a Shape>) -> Self {
		let mut inner = ffi::make_empty();
		for shape in shapes {
			ffi::compound_add(inner.pin_mut(), &shape.inner);
		}
		Shape::new(inner)
	}

	/// Every sub-shape of `kind`, in traversal order, as carriers sharing the
	/// handles of this shape.
	pub fn components(&self, kind: ShapeKind) -> Vec<Shape> {
		ffi::decompose_by_kind(&self.inner, kind.code()).iter().map(|found| Shape::new(ffi::clone_shape_handle(found))).collect()
	}

	pub fn kind(&self) -> ShapeKind {
		ShapeKind::of(&self.inner)
	}

	/// The `TopoDS_TShape*` behind the handle. Two handles that share a
	/// TShape -- a placed copy and its source -- share the id.
	pub fn id(&self) -> u64 {
		self.key().tshape
	}

	pub fn is_null(&self) -> bool {
		ffi::shape_is_null(&self.inner)
	}

	/// Native topological closure: a wire has no free vertex; a shell has
	/// no free boundary edge.
	pub fn is_closed(&self) -> bool {
		ffi::shape_is_closed(&self.inner)
	}

	/// What OCCT's `BRepCheck_Analyzer` reports.
	pub fn is_valid(&self) -> Result<bool, Error> {
		ffi::shape_is_valid(&self.inner).map_err(|error| Error::Validation(error.to_string()))
	}

	pub fn volume(&self) -> Result<f64, Error> {
		ffi::shape_volume(&self.inner).map_err(|error| Error::Properties(error.to_string()))
	}

	pub fn area(&self) -> Result<f64, Error> {
		ffi::shape_surface_area(&self.inner).map_err(|error| Error::Properties(error.to_string()))
	}

	pub fn center(&self) -> Result<DVec3, Error> {
		let (mut x, mut y, mut z) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_center_of_mass(&self.inner, &mut x, &mut y, &mut z).map_err(|error| Error::Properties(error.to_string()))?;
		Ok(DVec3::new(x, y, z))
	}

	pub fn inertia(&self) -> Result<DMat3, Error> {
		let (mut m00, mut m01, mut m02) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut m10, mut m11, mut m12) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut m20, mut m21, mut m22) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_inertia_tensor(&self.inner, &mut m00, &mut m01, &mut m02, &mut m10, &mut m11, &mut m12, &mut m20, &mut m21, &mut m22).map_err(|error| Error::Properties(error.to_string()))?;
		// OCCT fills row-major; `from_cols_array` is column-major.
		Ok(DMat3::from_cols_array(&[m00, m10, m20, m01, m11, m21, m02, m12, m22]))
	}

	pub fn contains(&self, point: DVec3) -> bool {
		ffi::shape_contains_point(&self.inner, point.x, point.y, point.z)
	}

	pub fn bounding_box(&self) -> [DVec3; 2] {
		let (mut xmin, mut ymin, mut zmin) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut xmax, mut ymax, mut zmax) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_bounding_box(&self.inner, &mut xmin, &mut ymin, &mut zmin, &mut xmax, &mut ymax, &mut zmax);
		[DVec3::new(xmin, ymin, zmin), DVec3::new(xmax, ymax, zmax)]
	}

	/// Edges in connected wire order, preserving each edge's traversal orientation.
	pub fn ordered_wire_edges(&self) -> Result<Vec<Edge>, Error> {
		ffi::wire_ordered_edges(&self.inner).map_err(|error| Error::Validation(error.to_string()))?.iter().map(|edge| Edge::try_from_ffi(ffi::clone_edge_handle(edge), "ordered wire edge is null".into())).collect()
	}

	/// The planar face a closed wire bounds, with the plane it lies on and,
	/// for every edge of the wire, the key of the face's own copy of it.
	/// `None` when the wire is open, not planar within `tolerance`, or bounds
	/// no area.
	pub fn planar_region(&self, tolerance: Option<f64>) -> Result<Option<PlanarRegion>, Error> {
		let (mut ox, mut oy, mut oz, mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		let mut edges = Vec::new();
		let face = ffi::wire_planar_region(&self.inner, tolerance.unwrap_or(f64::NAN), &mut ox, &mut oy, &mut oz, &mut nx, &mut ny, &mut nz, &mut edges).map_err(|error| Error::Validation(error.to_string()))?;
		if face.is_null() {
			return Ok(None);
		}
		let edges = edges.chunks_exact(4).map(|words| (Key { tshape: words[0], location: words[1] }, Key { tshape: words[2], location: words[3] })).collect();
		Ok(Some(PlanarRegion { face: Shape::new(face), origin: DVec3::new(ox, oy, oz), normal: DVec3::new(nx, ny, nz), edges }))
	}

	pub fn planar_region_coincides(&self, other: &Shape) -> Result<bool, Error> {
		ffi::planar_regions_coincide(&self.inner, &other.inner).map_err(|error| Error::Validation(error.to_string()))
	}

	pub fn iter_edge(&self) -> impl Iterator<Item = &Edge> + '_ {
		self.edges.get_or_init(|| ffi::shape_edges(&self.inner).iter().map(|edge| Edge::try_from_ffi(ffi::clone_edge_handle(edge), "shape_edges: null".into()).expect("shape_edges: unexpected null (this is a bug)")).collect()).iter()
	}

	pub fn iter_face(&self) -> impl Iterator<Item = &Face> + '_ {
		self.faces.get_or_init(|| ffi::shape_faces(&self.inner).iter().map(|face| Face::new(ffi::clone_face_handle(face))).collect()).iter()
	}

	/// The loops of edges only one face is adjacent to -- none for a closed
	/// shell. Loops are reported whole: a vertex where more than two boundary
	/// edges meet does not split them.
	pub fn free_boundaries(&self) -> Result<Vec<Vec<Edge>>, Error> {
		let mut loop_sizes: Vec<u32> = Vec::new();
		let edges = ffi::free_boundary_edges(&self.inner, false, false, &mut loop_sizes).map_err(|error| Error::Validation(error.to_string()))?;
		let mut taken = edges.iter();
		loop_sizes.into_iter().map(|size| (0..size).map(|_| Edge::try_from_ffi(ffi::clone_edge_handle(taken.next().expect("as many edges as the loop sizes count")), "free boundaries: null edge".into())).collect()).collect()
	}

	/// Moved by a translation: the handle is relocated, nothing is rebuilt,
	/// and the id is kept.
	pub fn translated(&self, translation: DVec3) -> Shape {
		Shape::new(ffi::transform_translate(&self.inner, translation.x, translation.y, translation.z))
	}

	/// Moved by a rotation about an axis, in radians; a relocation like
	/// [`Shape::translated`].
	pub fn rotated(&self, axis_origin: DVec3, axis_direction: DVec3, angle: f64) -> Shape {
		Shape::new(ffi::transform_rotate(&self.inner, axis_origin.x, axis_origin.y, axis_origin.z, axis_direction.x, axis_direction.y, axis_direction.z, angle))
	}

	/// A copy sharing no geometry with this shape; every id changes.
	pub fn deep_copy(&self) -> Shape {
		Shape::new(ffi::deep_copy(&self.inner))
	}

	/// Triangulate any shapes for display. Vertices are not shared between
	/// faces, and each triangle names the face it came from.
	pub fn mesh<'a>(shapes: impl IntoIterator<Item = &'a Shape>, options: Tessellation) -> Result<Mesh, Error> {
		let compound = Shape::compound(shapes);
		let data = ffi::mesh_shape(&compound.inner, options.deflection_linear, options.deflection_angular, options.relative_linear).map_err(|error| Error::Tessellation(error.to_string()))?;
		let vertex_count = data.vertices.len() / 3;
		let vertices: Vec<DVec3> = (0..vertex_count).map(|i| DVec3::new(data.vertices[i * 3], data.vertices[i * 3 + 1], data.vertices[i * 3 + 2])).collect();
		let normals: Vec<DVec3> = (0..vertex_count).map(|i| DVec3::new(data.normals[i * 3], data.normals[i * 3 + 1], data.normals[i * 3 + 2])).collect();
		let indices: Vec<usize> = data.indices.iter().map(|&i| i as usize).collect();
		let topology = compound.topology()?;
		let face_ids = data.face_indices.iter().map(|&face| topology.faces.get(face as usize).map(|fact| fact.key.tshape).ok_or_else(|| Error::Tessellation("triangle names no face of the report".into()))).collect::<Result<Vec<_>, _>>()?;

		// The triangulation's own edge polylines, NaN-separated, in report order.
		let mut edges: Vec<DVec3> = Vec::new();
		let mut edge_ranges = Vec::new();
		let points: Vec<DVec3> = data.edges.chunks_exact(3).map(|point| DVec3::new(point[0], point[1], point[2])).collect();
		for range in data.edge_ranges.chunks_exact(2) {
			let polyline = points.get(range[0] as usize..range[1] as usize).ok_or_else(|| Error::Tessellation("edge polyline range out of bounds".into()))?;
			if polyline.is_empty() {
				edge_ranges.push([edges.len(), edges.len()]);
				continue;
			}
			if !edges.is_empty() {
				edges.push(DVec3::NAN);
			}
			let start = edges.len();
			edges.extend_from_slice(polyline);
			edge_ranges.push([start, edges.len()]);
		}

		Ok(Mesh {
			vertices,
			normals,
			indices,
			face_ids,
			face_indices: data.face_indices,
			#[cfg(feature = "color")]
			colormap: Default::default(),
			edges,
			edge_ranges,
		})
	}

	/// The whole BRep payload of `reader` as one shape, and the byte at which
	/// the payload ended -- what a trailer reader looks past.
	pub(crate) fn read_brep_payload<R: Read>(reader: &mut R) -> Result<(Shape, Vec<u8>, usize), Error> {
		// Buffered whole: `BinTools::Read` seeks backwards to resolve shared
		// sub-shape references, so it cannot run off a sequential stream.
		let mut buffer = Vec::new();
		reader.read_to_end(&mut buffer)?;
		let mut consumed = 0usize;
		let inner = ffi::read_brep_stream(&buffer, &mut consumed);
		if inner.is_null() {
			return Err(Error::Io(std::io::Error::new(std::io::ErrorKind::InvalidData, "brep: reader produced no shape (invalid or corrupted input)")));
		}
		Ok((Shape::new(inner), buffer, consumed))
	}

	/// The BRep payload of `reader` as one shape, whatever kinds it holds.
	pub fn read_brep<R: Read>(reader: &mut R) -> Result<Shape, Error> {
		Self::read_brep_payload(reader).map(|(shape, _, _)| shape)
	}

	/// Write shapes as one BRep payload.
	pub fn write_brep<'a, W: Write>(shapes: impl IntoIterator<Item = &'a Shape>, writer: &mut W) -> Result<(), Error> {
		let mut rust_writer = RustWriter::from_ref(writer);
		if ffi::write_brep_stream(&Shape::compound(shapes).inner, &mut rust_writer) {
			Ok(())
		} else {
			Err(Error::Io(std::io::Error::other("brep: OCCT writer reported failure")))
		}
	}

	/// Read a STEP stream as one shape, without colours.
	pub fn read_step<R: Read>(reader: &mut R) -> Result<Shape, Error> {
		let mut rust_reader = RustReader::from_ref(reader);
		let inner = ffi::read_step_stream(&mut rust_reader);
		if inner.is_null() {
			return Err(Error::Io(std::io::Error::new(std::io::ErrorKind::InvalidData, "step: reader produced no shape (invalid or corrupted input)")));
		}
		Ok(Shape::new(inner))
	}

	/// Write shapes as one STEP stream, without colours.
	pub fn write_step<'a, W: Write>(shapes: impl IntoIterator<Item = &'a Shape>, writer: &mut W) -> Result<(), Error> {
		let mut rust_writer = RustWriter::from_ref(writer);
		if ffi::write_step_stream(&Shape::compound(shapes).inner, &mut rust_writer) {
			Ok(())
		} else {
			Err(Error::Io(std::io::Error::other("step: OCCT writer reported failure")))
		}
	}
}

/// A shallow clone: the same TShape under a new handle with fresh caches, so
/// the id -- and every face and edge id -- is kept. A copy with its own
/// geometry is [`Shape::deep_copy`].
impl Clone for Shape {
	fn clone(&self) -> Self {
		Shape::new(ffi::clone_shape_handle(&self.inner))
	}
}

impl std::fmt::Debug for Shape {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "Shape({:?}, {}, faces={}, edges={})", self.kind(), self.id(), self.iter_face().count(), self.iter_edge().count())
	}
}
