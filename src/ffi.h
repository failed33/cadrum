#pragma once

#include "rust/cxx.h"

// Types used directly in function signatures — keep minimal so that
// the cxx-generated bridge objects do not compile heavy OCCT headers.
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>

namespace cadrum {

// Type aliases to bring OCCT global types into cadrum namespace.
// Required because the cxx bridge uses namespace = "cadrum".
using TopoDS_Shape = ::TopoDS_Shape;
using TopoDS_Face = ::TopoDS_Face;
using TopoDS_Edge = ::TopoDS_Edge;

// Forward-declare the Rust opaque types (defined by cxx in ffi.rs.h)
struct RustReader;
struct RustWriter;

// Forward-declare shared structs (defined by cxx in ffi.rs.h)
struct MeshData;

bool shape_is_valid(const TopoDS_Shape& shape);

// ==================== Shape I/O (streambuf callback) ====================

// Plain STEP I/O — only built without FEATURE_COLOR; with color, STEP goes
// through XCAF (`read_step_color_stream` etc.) instead.
#ifndef FEATURE_COLOR
std::unique_ptr<TopoDS_Shape> read_step_stream(RustReader& reader);
bool write_step_stream(const TopoDS_Shape& shape, RustWriter& writer);
#endif
// `out_consumed` = length of the BinTools payload, where Rust's color trailer
// begins. Written ONLY on success; on failure nullptr comes back and it is untouched.
std::unique_ptr<TopoDS_Shape> read_brep_stream(
    rust::Slice<const uint8_t> data, size_t& out_consumed);
bool write_brep_stream(const TopoDS_Shape& shape, RustWriter& writer);

// ==================== Shape Constructors ====================

std::unique_ptr<TopoDS_Shape> make_empty();
std::unique_ptr<TopoDS_Shape> deep_copy(const TopoDS_Shape& shape);

// ==================== Builders (solid → solid with history) ====================
//
// Functions in this section take one or more solid inputs, rebuild topology,
// and append flat [post_id, src_id, ...] face derivation pairs to
// `out_history`. The Rust side stores these in `Solid::history`.

// ==================== Placements (a moved handle, no rebuild) ====================
//
// 3D transforms. translate/rotate use TopLoc_Location and preserve TShape*
// (Rust side keeps colormap and history intact). scale/mirror rebuild
// topology via BRepBuilderAPI_Transform; OCCT does not expose a face
// derivation table, so out_history is intentionally absent and the Rust
// side clears Solid::history (colormap is remapped by face order instead).

std::unique_ptr<TopoDS_Shape> transform_translate(
    const TopoDS_Shape& shape, double tx, double ty, double tz);
std::unique_ptr<TopoDS_Shape> transform_rotate(
    const TopoDS_Shape& shape,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    double angle);

// ==================== Shape Queries ====================

bool shape_is_null(const TopoDS_Shape& shape);
// `BRep_Tool::IsClosed`: every edge of the shape is shared by two faces.
bool shape_is_closed(const TopoDS_Shape& shape);
std::unique_ptr<std::vector<TopoDS_Edge>> wire_ordered_edges(const TopoDS_Shape& shape);
bool edge_is_reversed(const TopoDS_Edge& edge);
std::unique_ptr<TopoDS_Shape> wire_planar_region(const TopoDS_Shape& shape, double tolerance,
    double& ox, double& oy, double& oz, double& nx, double& ny, double& nz, rust::Vec<uint64_t>& out_edges);
bool planar_regions_coincide(const TopoDS_Shape& left, const TopoDS_Shape& right);
// Topological kind as a stable code, mirrored by Rust's `ShapeKind`:
// 0 null, 1 compound, 2 compsolid, 3 solid, 4 shell, 5 face, 6 wire, 7 edge,
// 8 vertex, 9 anything else. Mapped explicitly so the wire format does not
// inherit OCCT's own enum ordering.
uint32_t shape_kind(const TopoDS_Shape& shape);
double shape_volume(const TopoDS_Shape& shape);
double shape_surface_area(const TopoDS_Shape& shape);
void shape_center_of_mass(const TopoDS_Shape& shape,
    double& x, double& y, double& z);
void shape_inertia_tensor(const TopoDS_Shape& shape,
    double& m00, double& m01, double& m02,
    double& m10, double& m11, double& m12,
    double& m20, double& m21, double& m22);
bool shape_contains_point(const TopoDS_Shape& shape, double x, double y, double z);
void shape_bounding_box(const TopoDS_Shape& shape,
    double& xmin, double& ymin, double& zmin,
    double& xmax, double& ymax, double& zmax);

// ==================== Compound Decompose/Compose ====================

// Every sub-shape of the given `shape_kind` code, the shape itself included when
// it already has that kind. A code that names no topology (null, other) yields an
// empty vector, so the caller needs no special case.
std::unique_ptr<std::vector<TopoDS_Shape>> decompose_by_kind(const TopoDS_Shape& shape, uint32_t kind);
void compound_add(TopoDS_Shape& compound, const TopoDS_Shape& child);

// ==================== Meshing ====================

MeshData mesh_shape(const TopoDS_Shape& shape, double linear, double angular, bool relative);

// ==================== Topology report (topology.cpp) ====================

// Located identity: the TShape and the hash of the handle's location, the
// pair `TopoDS_Shape::IsSame` compares. Internal helpers first, then the
// bridge entry points.
uint64_t shape_tshape(const TopoDS_Shape& shape);
uint64_t shape_location(const TopoDS_Shape& shape);
void shape_key(const TopoDS_Shape& shape, uint64_t& tshape, uint64_t& location);
void face_key(const TopoDS_Face& face, uint64_t& tshape, uint64_t& location);
void edge_key(const TopoDS_Edge& edge, uint64_t& tshape, uint64_t& location);

struct TopologyData;
struct NearestData;
TopologyData shape_topology(const TopoDS_Shape& shape);
NearestData shape_nearest(const TopoDS_Shape& shape, double x, double y, double z);
// Point and unit tangent at `distance` along the edge's forward
// parametrisation, into six doubles. False when the abscissa cannot be placed.
bool edge_at_length(const TopoDS_Edge& edge, double distance, rust::Slice<double> out);

// ==================== Topology enumeration ====================

// One-shot enumeration of unique sub-shapes. `shape_edges` deduplicates
// edges shared between faces (so each edge appears exactly once).
// Callers typically cache the result in a Rust-side OnceLock<Vec<Edge>>.
std::unique_ptr<std::vector<TopoDS_Edge>> shape_edges(const TopoDS_Shape& shape);
std::unique_ptr<std::vector<TopoDS_Face>> shape_faces(const TopoDS_Shape& shape);

// One-shot enumeration of the boundary edges of a single face. Edges shared
// between this face's wires are deduplicated so each edge appears once.
std::unique_ptr<std::vector<TopoDS_Edge>> face_edges(const TopoDS_Face& face);

// Shallow handle clone — C++ copy-ctor shares the underlying TShape via
// OCCT's ref count. Needed when Rust materializes owned `Shape` / `Edge` /
// `Face` wrappers from the `&TopoDS_*` references yielded by
// `CxxVector::iter()`. Distinct from `deep_copy` / `deep_copy_edge` which
// create new TShapes.
std::unique_ptr<TopoDS_Shape> clone_shape_handle(const TopoDS_Shape& shape);
std::unique_ptr<TopoDS_Edge> clone_edge_handle(const TopoDS_Edge& edge);
std::unique_ptr<TopoDS_Face> clone_face_handle(const TopoDS_Face& face);

// ==================== Edge Methods ====================

// Approximate an edge as a polyline. Takes independent angular/chord
// deflection bounds. Returns a flat xyz `Vec<f64>` (length = 3 * point count).
rust::Vec<double> edge_approximation_segments(
    const TopoDS_Edge& edge, double linear, double angular, bool relative);

// Construct a single helical edge on a cylindrical surface centered at the
// world origin. `axis` is the cylinder axis direction; `x_ref` is the
// reference direction that anchors the local +X axis of the cylindrical
// frame. The helix starts at `radius * normalize(x_ref - project_on(axis))`
// (i.e. at the +X side of the local frame, z=0) and rises by `height` over
// `height/pitch` turns. `x_ref` must not be parallel to `axis`.
std::unique_ptr<TopoDS_Edge> make_helix_edge(
    double ax, double ay, double az,
    double xrx, double xry, double xrz,
    double radius, double pitch, double height);

// Build a closed polygon from `coords` (flat xyz triples, ≥3 points) and
// return its constituent edges in order. The closing edge from the last
// point back to the first is included.
std::unique_ptr<std::vector<TopoDS_Edge>> make_polygon_edges(
    rust::Slice<const double> coords);

// Construct a closed circular edge of `radius` centered at the world origin,
// lying in the plane normal to `axis`. The local +X axis of the circle's
// frame (which determines the parametric start point) is chosen by OCCT
// from an arbitrary orthogonal direction to `axis`.
std::unique_ptr<TopoDS_Edge> make_circle_edge(
    double ax, double ay, double az, double radius);

// Construct a straight line segment edge from point a to point b.
std::unique_ptr<TopoDS_Edge> make_line_edge(
    double ax, double ay, double az,
    double bx, double by, double bz);

// Construct a circular arc edge through three points (start, mid, end).
// `mid` must not be collinear with `start` and `end`. On degenerate input
// OCCT returns nullptr.
std::unique_ptr<TopoDS_Edge> make_arc_edge(
    double sx, double sy, double sz,
    double mx, double my, double mz,
    double ex, double ey, double ez);

// Cubic B-spline edge interpolating data points.
//
// `coords` is a flat array of xyz triples (length must be a multiple of 3
// and ≥ 6). Each (x, y, z) is one interpolation target — the resulting
// curve passes through every input point exactly. `end_kind` selects the
// end-condition variant of `BSplineEnd`:
//   0 = Periodic (C² periodic; tangent args ignored)
//   1 = NotAKnot (open, OCCT default; tangent args ignored)
//   2 = Clamped  (open, explicit start/end tangents in (sx,sy,sz)/(ex,ey,ez))
// Returns nullptr on any failure.
std::unique_ptr<TopoDS_Edge> make_bspline_edge(
    rust::Slice<const double> coords,
    uint32_t end_kind,
    double sx, double sy, double sz,
    double ex, double ey, double ez, double tolerance);

// Edge query helpers.
void edge_endpoints(const TopoDS_Edge& edge,
    double& sx, double& sy, double& sz,
    double& ex, double& ey, double& ez);
void edge_tangents(const TopoDS_Edge& edge,
    double& sx, double& sy, double& sz,
    double& ex, double& ey, double& ez);
bool edge_is_closed(const TopoDS_Edge& edge);

// Project a world point onto the edge's underlying curve. Returns false if
// the curve is missing or the projector cannot converge (leaves outputs 0).
bool edge_project_point(const TopoDS_Edge& edge,
    double px, double py, double pz,
    double& cpx, double& cpy, double& cpz,
    double& tx, double& ty, double& tz);

// Edge clone (deep copy of underlying TShape).
std::unique_ptr<TopoDS_Edge> deep_copy_edge(const TopoDS_Edge& edge);

// Edge spatial transforms. Mirror the shape-level helpers but operate on
// TopoDS_Edge directly so the Rust wrapper can stay edge-typed.
std::unique_ptr<TopoDS_Edge> translate_edge(
    const TopoDS_Edge& edge, double tx, double ty, double tz);
std::unique_ptr<TopoDS_Edge> rotate_edge(
    const TopoDS_Edge& edge,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    double angle);
std::unique_ptr<TopoDS_Edge> scale_edge(
    const TopoDS_Edge& edge,
    double cx, double cy, double cz,
    double factor);
std::unique_ptr<TopoDS_Edge> mirror_edge(
    const TopoDS_Edge& edge,
    double ox, double oy, double oz,
    double nx, double ny, double nz);

// Helpers for the Rust side to construct the input vector of `apply_algorithm`.
// Edges and faces are shapes too; pushing them as such keeps the handle (and
// so the id the history reports) the caller holds.
std::unique_ptr<std::vector<TopoDS_Shape>> shape_vec_new();
void shape_vec_push(std::vector<TopoDS_Shape>& v, const TopoDS_Shape& s);
void shape_vec_push_edge(std::vector<TopoDS_Shape>& v, const TopoDS_Edge& e);
void shape_vec_push_face(std::vector<TopoDS_Shape>& v, const TopoDS_Face& f);

// ==================== The algorithm table ====================
// One entry point over every OCCT algorithm the binding offers; the row codes
// and the layout of `shapes` / `scalars` / `integers` per row are stated by
// `occt::algorithm::Algorithm` in Rust and mirrored in ffi.cpp. `out_lineage`
// receives five words per descent (relation, result key, source key) and
// `out_landmarks` three per landmark (role, face key); the codes are stated
// by `occt::algorithm`. Failure is a `std::runtime_error` naming the row;
// never a null shape.
std::unique_ptr<TopoDS_Shape> apply_algorithm(
    uint32_t algorithm,
    const std::vector<TopoDS_Shape>& shapes,
    rust::Slice<const double> scalars,
    rust::Slice<const int64_t> integers,
    rust::Vec<uint64_t>& out_lineage,
    rust::Vec<uint64_t>& out_landmarks,
    bool& out_refused);

// Free (single-adjacent) boundaries of an already sewn shape, as edges grouped
// into loops: `out_loop_sizes` holds the edge count of each loop, in order.
// `split_closed` / `split_open` are ShapeAnalysis_FreeBounds' wire splitting
// controls; false/false keeps every loop whole.
std::unique_ptr<std::vector<TopoDS_Edge>> free_boundary_edges(
    const TopoDS_Shape& shape,
    bool split_closed,
    bool split_open,
    rust::Vec<uint32_t>& out_loop_sizes);

// Build a B-spline surface solid from a 2D point grid.
// `coords` is a flat array of xyz triples, length = 3 * nu * nv.
// V direction (cross-section, j index) is always periodic.
// U direction (longitudinal, i index) is periodic iff `u_periodic=true`
// (producing a torus); otherwise the U-ends are capped with planar faces
// (producing a pipe). Returns nullptr on any OCCT failure.
std::unique_ptr<TopoDS_Shape> make_bspline_solid(
    rust::Slice<const double> coords,
    uint32_t nu, uint32_t nv,
    bool u_periodic);

// ==================== Face Methods ====================

// Project a 3D point onto `face`. Sister of `edge_project_point`.
// Returns the closest point on the (trimmed) face surface and the outward
// face normal there. `nx/ny/nz` is the zero vector when the projector
// cannot define a normal at the closest hit (degenerate surface point).
// Returns false on catastrophic OCCT failure.
bool face_project_point(const TopoDS_Face& face,
    double px, double py, double pz,
    double& cpx, double& cpy, double& cpz,
    double& nx, double& ny, double& nz);


std::unique_ptr<TopoDS_Shape> make_bspline_solid_with_tolerance(
    rust::Slice<const double> coords, uint32_t nu, uint32_t nv,
    bool u_periodic, double tolerance, double sewing_tolerance);

void face_sample(const TopoDS_Face& face, double u_fraction, double v_fraction, rust::Slice<double> result);

} // namespace cadrum

#include <Standard_Failure.hxx>

namespace rust::behavior {
template <typename Try, typename Fail>
static void trycatch(Try&& func, Fail&& fail) noexcept {
    try {
        func();
    } catch (const Standard_Failure& error) {
        fail(error.what());
    } catch (const std::exception& error) {
        fail(error.what());
    }
}
} // namespace rust::behavior

#ifdef FEATURE_COLOR

namespace cadrum {

// ==================== Colored STEP I/O ====================

// `out_ids` = TShape* of each colored sub-shape, `out_rgb` = flat [r,g,b,...] in
// OCC native space. An id is a FACE's or a SOLID's — a styled_item targets either.
// Returns nullptr on failure.
std::unique_ptr<TopoDS_Shape> read_step_color_stream(
    RustReader&          reader,
    rust::Vec<uint64_t>& out_ids,
    rust::Vec<float>&    out_rgb);

// A solid id is written as one styled_item on that solid; a face style, being the
// more specific one, overrides it.
bool write_step_color_stream(
    const TopoDS_Shape&         shape,
    rust::Slice<const uint64_t> ids,
    rust::Slice<const float>    rgb,
    RustWriter&                 writer);

} // namespace cadrum

#endif // FEATURE_COLOR
