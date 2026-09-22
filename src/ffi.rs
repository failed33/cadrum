use std::io::{Read, Write};

#[cxx::bridge(namespace = "cadrum")]
mod ffi_bridge {
	// Shared struct for mesh data returned from C++
	struct MeshData {
		vertices: Vec<f64>, // flat xyz
		normals: Vec<f64>,  // flat xyz, one per vertex
		indices: Vec<u32>,
		face_indices: Vec<u32>, // per-triangle face position in the topology report
		edges: Vec<f64>,        // flat xyz, the triangulation's own edge polylines
		edge_ranges: Vec<u32>,  // [start, end) point range per edge, in report order
	}

	// ==================== The topology report ====================
	//
	// Every face, edge and vertex in `TopExp::MapShapes` order: one row per
	// sub-shape in each `face_*`, `edge_*` and `vertex_*` column. The geometry
	// a face or edge lies on is one row of the definition column its kind
	// names, at `face_definition` / `edge_definition`; a kind without a column
	// (other, degenerate) leaves that index unread. Incidence lists are CSR
	// with one more offset than rows; an absent vertex index is `u32::MAX`.
	// These structs are the schema: C++ fills them by field name and Rust
	// parses them once, so no side counts words.

	struct Xyz {
		x: f64,
		y: f64,
		z: f64,
	}
	// `gp_Ax3` less its derived y axis: where an analytic surface or a conic
	// stands and how it is parametrised.
	struct PlacementData {
		origin: Xyz,
		axis: Xyz,
		reference: Xyz,
	}
	struct AxisData {
		origin: Xyz,
		direction: Xyz,
	}
	struct KeyData {
		tshape: u64,
		location: u64,
	}

	// `GeomAbs_SurfaceType`, as this bridge numbers it.
	enum SurfaceCode {
		Plane,
		Cylinder,
		Cone,
		Sphere,
		Torus,
		Bezier,
		BSpline,
		Revolution,
		Extrusion,
		Offset,
		Other,
	}
	// `GeomAbs_CurveType`, as this bridge numbers it, plus the edge that has
	// no curve at all.
	enum CurveCode {
		Line,
		Circle,
		Ellipse,
		Hyperbola,
		Parabola,
		Bezier,
		BSpline,
		Offset,
		Other,
		Degenerate,
	}

	struct PlaneDef {
		placement: PlacementData,
	}
	struct CylinderDef {
		placement: PlacementData,
		radius: f64,
	}
	struct ConeDef {
		placement: PlacementData,
		radius: f64,
		semi_angle: f64,
	}
	struct SphereDef {
		placement: PlacementData,
		radius: f64,
	}
	struct TorusDef {
		placement: PlacementData,
		major_radius: f64,
		minor_radius: f64,
	}
	struct SplineSurfaceDef {
		u_degree: u32,
		v_degree: u32,
		u_poles: u32,
		v_poles: u32,
		u_periodic: bool,
		v_periodic: bool,
		rational: bool,
	}
	struct RevolutionDef {
		axis: AxisData,
	}
	struct ExtrusionDef {
		direction: Xyz,
	}
	struct OffsetDef {
		offset: f64,
	}
	struct LineDef {
		axis: AxisData,
	}
	struct CircleDef {
		placement: PlacementData,
		radius: f64,
	}
	struct EllipseDef {
		placement: PlacementData,
		major_radius: f64,
		minor_radius: f64,
	}
	struct HyperbolaDef {
		placement: PlacementData,
		major_radius: f64,
		minor_radius: f64,
	}
	struct ParabolaDef {
		placement: PlacementData,
		focal: f64,
	}
	struct SplineCurveDef {
		degree: u32,
		poles: u32,
		periodic: bool,
		rational: bool,
	}
	// The ends of an edge's curve in its forward parametrisation.
	struct EdgeEnds {
		start: Xyz,
		start_tangent: Xyz,
		end: Xyz,
		end_tangent: Xyz,
	}
	struct ExtentData {
		low: Xyz,
		high: Xyz,
	}

	struct TopologyData {
		face_keys: Vec<KeyData>,
		face_surface: Vec<SurfaceCode>,
		face_definition: Vec<u32>,
		face_reversed: Vec<bool>,
		face_tolerance: Vec<f64>,
		face_edge_offsets: Vec<u32>,
		face_edges: Vec<u32>,
		edge_keys: Vec<KeyData>,
		edge_curve: Vec<CurveCode>,
		edge_definition: Vec<u32>,
		edge_ends: Vec<EdgeEnds>, // zero, unread, for a degenerate edge
		edge_length: Vec<f64>,
		edge_tolerance: Vec<f64>,
		edge_vertices: Vec<u32>, // start, end
		edge_face_offsets: Vec<u32>,
		edge_faces: Vec<u32>,
		vertex_keys: Vec<KeyData>,
		vertex_points: Vec<Xyz>,
		vertex_tolerance: Vec<f64>,
		planes: Vec<PlaneDef>,
		cylinders: Vec<CylinderDef>,
		cones: Vec<ConeDef>,
		spheres: Vec<SphereDef>,
		tori: Vec<TorusDef>,
		spline_surfaces: Vec<SplineSurfaceDef>,
		revolutions: Vec<RevolutionDef>,
		extrusions: Vec<ExtrusionDef>,
		surface_offsets: Vec<OffsetDef>,
		lines: Vec<LineDef>,
		circles: Vec<CircleDef>,
		ellipses: Vec<EllipseDef>,
		hyperbolas: Vec<HyperbolaDef>,
		parabolas: Vec<ParabolaDef>,
		spline_curves: Vec<SplineCurveDef>,
		curve_offsets: Vec<OffsetDef>,
		// The axis-aligned extent of the geometry itself; `bounded` is false
		// for a shape with nothing to measure, whose `extent` is unread.
		bounded: bool,
		extent: ExtentData,
	}

	// The closest point of a shape to a probe: `support` is 0 for none,
	// 1 vertex, 2 edge, 3 face; the normal is NaN unless the support is a face.
	struct NearestData {
		support: u32,
		tshape: u64,
		location: u64,
		px: f64,
		py: f64,
		pz: f64,
		nx: f64,
		ny: f64,
		nz: f64,
	}

	// Expose Rust stream types to C++ for streambuf callbacks
	extern "Rust" {
		type RustReader;
		type RustWriter;

		fn rust_reader_read(reader: &mut RustReader, buf: &mut [u8]) -> usize;
		fn rust_writer_write(writer: &mut RustWriter, buf: &[u8]) -> usize;
	}

	unsafe extern "C++" {
		include!("cadrum/src/ffi.h");

		// Opaque C++ types (accessed as cadrum::TopoDS_Shape etc. via using aliases)
		type TopoDS_Shape;
		type TopoDS_Face;
		type TopoDS_Edge;

		// ==================== Shape I/O (streambuf callback) ====================

		fn read_step_stream(reader: &mut RustReader) -> UniquePtr<TopoDS_Shape>;
		fn write_step_stream(shape: &TopoDS_Shape, writer: &mut RustWriter) -> bool;
		// `out_consumed` = payload length, where the color trailer begins. Written only
		// when the returned pointer is non-null.
		fn read_brep_stream(data: &[u8], out_consumed: &mut usize) -> UniquePtr<TopoDS_Shape>;
		fn write_brep_stream(shape: &TopoDS_Shape, writer: &mut RustWriter) -> bool;

		// ==================== Shape Constructors ====================

		fn make_empty() -> UniquePtr<TopoDS_Shape>;

		fn deep_copy(shape: &TopoDS_Shape) -> UniquePtr<TopoDS_Shape>;

		// ==================== Placements (a moved handle, no rebuild) ====================

		fn transform_translate(shape: &TopoDS_Shape, tx: f64, ty: f64, tz: f64) -> UniquePtr<TopoDS_Shape>;

		fn transform_rotate(shape: &TopoDS_Shape, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, angle: f64) -> UniquePtr<TopoDS_Shape>;

		// ==================== Shape Queries ====================

		fn shape_is_null(shape: &TopoDS_Shape) -> bool;
		fn shape_is_closed(shape: &TopoDS_Shape) -> bool;
		fn wire_ordered_edges(shape: &TopoDS_Shape) -> Result<UniquePtr<CxxVector<TopoDS_Edge>>>;
		fn edge_is_reversed(edge: &TopoDS_Edge) -> bool;
		// `out_edges` receives four words per wire edge: its key, then the key
		// of the face's copy of it.
		fn wire_planar_region(shape: &TopoDS_Shape, tolerance: f64, ox: &mut f64, oy: &mut f64, oz: &mut f64, nx: &mut f64, ny: &mut f64, nz: &mut f64, out_edges: &mut Vec<u64>) -> Result<UniquePtr<TopoDS_Shape>>;
		fn planar_regions_coincide(left: &TopoDS_Shape, right: &TopoDS_Shape) -> Result<bool>;
		// Codes mirrored by `occt::shape::ShapeKind`; see ffi.h.
		fn shape_kind(shape: &TopoDS_Shape) -> u32;
		fn shape_is_valid(shape: &TopoDS_Shape) -> Result<bool>;
		fn shape_volume(shape: &TopoDS_Shape) -> Result<f64>;
		fn shape_surface_area(shape: &TopoDS_Shape) -> Result<f64>;
		fn shape_center_of_mass(shape: &TopoDS_Shape, x: &mut f64, y: &mut f64, z: &mut f64) -> Result<()>;
		fn shape_inertia_tensor(shape: &TopoDS_Shape, m00: &mut f64, m01: &mut f64, m02: &mut f64, m10: &mut f64, m11: &mut f64, m12: &mut f64, m20: &mut f64, m21: &mut f64, m22: &mut f64) -> Result<()>;
		fn shape_contains_point(shape: &TopoDS_Shape, x: f64, y: f64, z: f64) -> bool;
		fn shape_bounding_box(shape: &TopoDS_Shape, xmin: &mut f64, ymin: &mut f64, zmin: &mut f64, xmax: &mut f64, ymax: &mut f64, zmax: &mut f64);

		// ==================== Compound Decompose/Compose ====================

		fn decompose_by_kind(shape: &TopoDS_Shape, kind: u32) -> UniquePtr<CxxVector<TopoDS_Shape>>;
		fn compound_add(compound: Pin<&mut TopoDS_Shape>, child: &TopoDS_Shape);

		// ==================== Meshing ====================

		fn mesh_shape(shape: &TopoDS_Shape, linear: f64, angular: f64, relative: bool) -> Result<MeshData>;

		// ==================== Topology report ====================

		fn shape_topology(shape: &TopoDS_Shape) -> Result<TopologyData>;
		fn shape_nearest(shape: &TopoDS_Shape, x: f64, y: f64, z: f64) -> Result<NearestData>;
		fn edge_at_length(edge: &TopoDS_Edge, distance: f64, out: &mut [f64]) -> bool;
		fn shape_key(shape: &TopoDS_Shape, tshape: &mut u64, location: &mut u64);
		fn face_key(face: &TopoDS_Face, tshape: &mut u64, location: &mut u64);
		fn edge_key(edge: &TopoDS_Edge, tshape: &mut u64, location: &mut u64);

		// ==================== Topology enumeration ====================

		fn shape_edges(shape: &TopoDS_Shape) -> UniquePtr<CxxVector<TopoDS_Edge>>;
		fn shape_faces(shape: &TopoDS_Shape) -> UniquePtr<CxxVector<TopoDS_Face>>;
		fn face_edges(face: &TopoDS_Face) -> UniquePtr<CxxVector<TopoDS_Edge>>;

		fn clone_shape_handle(shape: &TopoDS_Shape) -> UniquePtr<TopoDS_Shape>;
		fn clone_edge_handle(edge: &TopoDS_Edge) -> UniquePtr<TopoDS_Edge>;
		fn clone_face_handle(face: &TopoDS_Face) -> UniquePtr<TopoDS_Face>;

		// ==================== Face Methods ====================

		fn face_sample(face: &TopoDS_Face, u_fraction: f64, v_fraction: f64, result: &mut [f64]) -> Result<()>;

		fn face_project_point(face: &TopoDS_Face, px: f64, py: f64, pz: f64, cpx: &mut f64, cpy: &mut f64, cpz: &mut f64, nx: &mut f64, ny: &mut f64, nz: &mut f64) -> bool;

		// ==================== Edge Methods ====================

		fn edge_approximation_segments(edge: &TopoDS_Edge, linear: f64, angular: f64, relative: bool) -> Vec<f64>;

		fn make_helix_edge(ax: f64, ay: f64, az: f64, xrx: f64, xry: f64, xrz: f64, radius: f64, pitch: f64, height: f64) -> UniquePtr<TopoDS_Edge>;
		fn make_polygon_edges(coords: &[f64]) -> UniquePtr<CxxVector<TopoDS_Edge>>;
		fn make_circle_edge(ax: f64, ay: f64, az: f64, radius: f64) -> UniquePtr<TopoDS_Edge>;
		fn make_line_edge(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> UniquePtr<TopoDS_Edge>;
		fn make_arc_edge(sx: f64, sy: f64, sz: f64, mx: f64, my: f64, mz: f64, ex: f64, ey: f64, ez: f64) -> UniquePtr<TopoDS_Edge>;
		fn make_bspline_edge(coords: &[f64], end_kind: u32, sx: f64, sy: f64, sz: f64, ex: f64, ey: f64, ez: f64, tolerance: f64) -> UniquePtr<TopoDS_Edge>;

		fn edge_endpoints(edge: &TopoDS_Edge, sx: &mut f64, sy: &mut f64, sz: &mut f64, ex: &mut f64, ey: &mut f64, ez: &mut f64);
		fn edge_tangents(edge: &TopoDS_Edge, sx: &mut f64, sy: &mut f64, sz: &mut f64, ex: &mut f64, ey: &mut f64, ez: &mut f64);
		fn edge_is_closed(edge: &TopoDS_Edge) -> bool;
		fn edge_project_point(edge: &TopoDS_Edge, px: f64, py: f64, pz: f64, cpx: &mut f64, cpy: &mut f64, cpz: &mut f64, tx: &mut f64, ty: &mut f64, tz: &mut f64) -> bool;

		fn deep_copy_edge(edge: &TopoDS_Edge) -> UniquePtr<TopoDS_Edge>;

		fn translate_edge(edge: &TopoDS_Edge, tx: f64, ty: f64, tz: f64) -> UniquePtr<TopoDS_Edge>;
		fn rotate_edge(edge: &TopoDS_Edge, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64, angle: f64) -> UniquePtr<TopoDS_Edge>;
		fn scale_edge(edge: &TopoDS_Edge, cx: f64, cy: f64, cz: f64, factor: f64) -> UniquePtr<TopoDS_Edge>;
		fn mirror_edge(edge: &TopoDS_Edge, ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64) -> UniquePtr<TopoDS_Edge>;

		// Free boundary loops of a shape; failure is a `cxx::Exception`.
		fn free_boundary_edges(shape: &TopoDS_Shape, split_closed: bool, split_open: bool, out_loop_sizes: &mut Vec<u32>) -> Result<UniquePtr<CxxVector<TopoDS_Edge>>>;
		fn make_bspline_solid(coords: &[f64], nu: u32, nv: u32, u_periodic: bool) -> UniquePtr<TopoDS_Shape>;

		fn shape_vec_new() -> UniquePtr<CxxVector<TopoDS_Shape>>;
		fn shape_vec_push(v: Pin<&mut CxxVector<TopoDS_Shape>>, s: &TopoDS_Shape);
		fn shape_vec_push_edge(v: Pin<&mut CxxVector<TopoDS_Shape>>, e: &TopoDS_Edge);
		fn shape_vec_push_face(v: Pin<&mut CxxVector<TopoDS_Shape>>, f: &TopoDS_Face);

		// ==================== The algorithm table ====================
		// See `occt::algorithm`. Failure is a `cxx::Exception` naming the row.
		// `out_lineage` holds five words per descent: relation, result key,
		// source key; `out_landmarks` three per landmark: role, face key.
		// `out_refused` is set when the row refused its input before OCCT ran
		// on it, so the failure is the caller's to correct.
		fn apply_algorithm(algorithm: u32, shapes: &CxxVector<TopoDS_Shape>, scalars: &[f64], integers: &[i64], out_lineage: &mut Vec<u64>, out_landmarks: &mut Vec<u64>, out_refused: &mut bool) -> Result<UniquePtr<TopoDS_Shape>>;

	}
}

// Re-export all bridge items so other modules can use `ffi::TopoDS_Shape` etc.
pub use ffi_bridge::*;

// ==================== Stream wrappers ====================
pub struct RustReader {
	inner: *mut dyn Read,
}

impl RustReader {
	/// Create a new RustReader wrapping the given reader.
	///
	/// # Safety
	/// The caller must ensure that the resulting `RustReader` is not used
	/// after `reader` is dropped. In practice, this is guaranteed because
	/// the C++ FFI call is synchronous.
	pub fn from_ref<'a>(reader: &'a mut (dyn Read + 'a)) -> Self {
		// SAFETY: Caller must ensure `reader` outlives this RustReader.
		// The `'static` bound is required by the raw pointer type, so we
		// use transmute to erase the lifetime (lifetimes are compile-time only).
		RustReader { inner: unsafe { std::mem::transmute::<*mut (dyn Read + 'a), *mut (dyn Read + 'static)>(reader as *mut (dyn Read + 'a)) } }
	}
}

/// Wrapper around `dyn Write` passed to C++ as an opaque extern Rust type.
///
/// C++ calls `rust_writer_write()` to push bytes into the Rust writer,
/// receiving them from a `std::streambuf` subclass that OCC writes to.
pub struct RustWriter {
	inner: *mut dyn Write,
}

impl RustWriter {
	/// Create a new RustWriter wrapping the given writer.
	///
	/// # Safety
	/// Same as `RustReader::from_ref`.
	pub fn from_ref<'a>(writer: &'a mut (dyn Write + 'a)) -> Self {
		// SAFETY: Caller must ensure `writer` outlives this RustWriter.
		// See RustReader::from_ref for the same rationale.
		RustWriter { inner: unsafe { std::mem::transmute::<*mut (dyn Write + 'a), *mut (dyn Write + 'static)>(writer as *mut (dyn Write + 'a)) } }
	}
}

/// FFI callback: read up to `buf.len()` bytes from the RustReader.
/// Returns the number of bytes actually read (0 = EOF).
pub fn rust_reader_read(reader: &mut RustReader, buf: &mut [u8]) -> usize {
	unsafe { (*reader.inner).read(buf).unwrap_or(0) }
}

/// FFI callback: write bytes into the RustWriter.
/// Returns the number of bytes actually written.
pub fn rust_writer_write(writer: &mut RustWriter, buf: &[u8]) -> usize {
	unsafe { (*writer.inner).write(buf).unwrap_or(0) }
}

// cxx opaque types default to `!Send + !Sync`. We mark them `Send` here so
// that `UniquePtr<TopoDS_Shape>` (and friends) become `Send`, which in turn
// makes our wrapper types (`Shape`, `Solid`, `Face`, `Edge`) auto-Send.
//
// Safety rationale:
//   - `UniquePtr` gives exclusive ownership — no aliasing is possible.
//   - These values are never shared across threads simultaneously; they are
//     only *moved* to another thread, which is what `Send` permits.
//   - `Sync` is intentionally NOT implemented: OCC's `Handle<Geom_XXX>`
//     reference counts are non-atomic, so concurrent `&T` access across
//     threads would be unsound.
unsafe impl Send for TopoDS_Shape {}
unsafe impl Send for TopoDS_Face {}
unsafe impl Send for TopoDS_Edge {}
