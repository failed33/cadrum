#include "cadrum/src/ffi.rs.h"

// ==================== OCCT headers (impl only — not exposed via wrapper.h) ====================
//
// Grouped by responsibility. Anything used in wrapper.h is included there;
// here we only pull in what the implementations need.

// --- Standard / exceptions ---
#include <Standard_Failure.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Law_Interpol.hxx>
#include <limits>
#include <stdexcept>
#include <string>

// --- Topology types & navigation ---
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>

// --- Geometry primitives (gp / Geom / 2d) ---
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom2d_Line.hxx>
#include <GC_MakeArcOfCircle.hxx>

// --- BRep builders (faces / wires / edges / solid primitives) ---
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <gp_GTrsf.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_ExtPF.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSweep.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BOPAlgo_Operation.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepProj_Projection.hxx>

// --- Boolean operations & shape cleanup ---
#include <BOPAlgo_CellsBuilder.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepTools_History.hxx>

// --- Sweep / pipe / loft ---
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <BRepOffset_Mode.hxx>
#include <GeomAbs_JoinType.hxx>
#include <GeomAbs_Shape.hxx>

// --- Mesh, classification, mass / surface properties ---
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_MergeNodesTool.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

// --- Curve adaptation / approximation ---
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_HArray1.hxx>
#include <Precision.hxx>

// --- I/O (BREP / STEP / progress) ---
// STEP-specific headers are only needed by the non-color STEP path
// (`read_step_stream` / `write_step_stream`); with color, STEP routes
// through XCAF in the FEATURE_COLOR section below.
#include <BinTools.hxx>
#ifndef FEATURE_COLOR
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Message_ProgressRange.hxx>
#endif
#include <Message.hxx>

// --- Signal translation ---
#include <OSD.hxx>
#include <OSD_Exception_ACCESS_VIOLATION.hxx>
#include <Standard_NumericError.hxx>
#if defined(__unix__) || defined(__APPLE__)
#include <csignal>
#include <pthread.h>
#endif

// --- C++ standard library ---
#include <mutex>
#include <istream>
#include <ostream>
#include <sstream>
#include <streambuf>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <cstdlib>
#include <set>
#include <type_traits>
#include <utility>

namespace cadrum {

// ==================== Signal-to-exception translation ====================
//
// OCCT algorithms fault on degenerate input (`BRepOffsetAPI_MakeOffsetShape`
// at wall thickness ≥ half the body extent, `BRepOffsetAPI_MakeFilling` over a
// boundary enclosing no area) instead of raising `Standard_Failure`, so every
// `catch (const Standard_Failure&)` below is unreachable and the process
// aborts. OCCT's own remedy is `OSD::SetSignal`, which translates the C signal
// into a `Standard_Failure`-derived exception — but `OSD_signal.cxx` is one of
// the POSIX sources `build.rs` body-stubs, so in the linked library
// `OSD::SetSignal` is a bare `ret` and installs nothing. The translation is
// therefore installed here. Outside a bridge call it throws the same exception
// types OCCT's own handler throws, so a catch block sees the failure it
// already expects.
//
// A throw out of a signal frame arrives at a catch only where every frame
// between the faulting instruction and that catch is unwindable. A `noexcept`
// member of the standard library, an assembly leaf, or -- on GCC targets, the
// prebuilt OCCT throughout -- a function compiled without exception tables over
// its faulting instructions terminates the process instead. So inside a bridge
// call the handler does not throw: it returns to the call's fault return
// (ffi.h), which reports the fault as that call's error. The throw remains for
// a fault outside every bridge call.
//
// Dispositions set by `sigaction` are process-wide and shared by every thread,
// so one installation covers callers that evaluate off the main thread; the
// per-thread part of OCCT's setup (`OSD::SetThreadLocalSignal`) only arms
// floating-point traps, which stay disarmed here as `OSD::SetSignal(false)`
// leaves them.
#if defined(__unix__) || defined(__APPLE__)

sigjmp_buf*& fault_return() {
    thread_local sigjmp_buf* live = nullptr;
    return live;
}

// Names the signal actually caught: a report that renames a bus error as a
// segmentation fault sends the reader after the wrong cause.
const char* fault_message(int signal_number) {
    switch (signal_number) {
        case SIGFPE:
            return "SIGFPE raised inside an OCCT algorithm";
        case SIGBUS:
            return "SIGBUS raised inside an OCCT algorithm";
        case SIGSEGV:
            return "SIGSEGV raised inside an OCCT algorithm";
        default:
            return "a fault signal raised inside an OCCT algorithm";
    }
}

#endif

namespace {

#if defined(__unix__) || defined(__APPLE__)

extern "C" void raise_signal_as_failure(int signal_number, siginfo_t*, void*) {
    // `sigsetjmp(here, 1)` saved the mask with the signal unblocked, and
    // `siglongjmp` restores it, so the jump leaves no `sigreturn` behind.
    if (sigjmp_buf* here = fault_return()) siglongjmp(*here, signal_number);

    // The signal is blocked for the duration of its own handler, and throwing
    // out of the handler skips the `sigreturn` that would restore the mask.
    sigset_t raised;
    sigemptyset(&raised);
    sigaddset(&raised, signal_number);
    pthread_sigmask(SIG_UNBLOCK, &raised, nullptr);

    if (signal_number == SIGFPE) throw Standard_NumericError(fault_message(signal_number));
    throw OSD_Exception_ACCESS_VIOLATION(fault_message(signal_number));
}

void install_signal_translation() {
    struct sigaction action = {};
    action.sa_sigaction = raise_signal_as_failure;
    action.sa_flags = SA_SIGINFO;
    sigemptyset(&action.sa_mask);
    for (int signal_number : {SIGSEGV, SIGBUS, SIGFPE}) {
        sigaction(signal_number, &action, nullptr);
    }
}

#else

// Windows and wasm have no POSIX disposition to set. `OSD::SetSignal` is the
// call OCCT intends for them, and takes effect the moment `build.rs` stops
// stubbing `OSD_signal.cxx`; until then those targets still abort on a fault.
void install_signal_translation() {
    OSD::SetSignal(false);
}

#endif

std::once_flag signal_translation_once;

/// Runs during load-time initialisation of this translation unit, before any
/// binding can be called; `std::call_once` keeps it single even if a host
/// loads the binding from several threads.
const bool signal_translation_installed = [] {
    std::call_once(signal_translation_once, install_signal_translation);
    return true;
}();

}  // namespace

bool shape_is_valid(const TopoDS_Shape& shape) {
    try {
        return !shape.IsNull() && BRepCheck_Analyzer(shape, true, false, true).IsValid();
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(error.what());
    }
}

// OCCT defaults to a stdout printer that emits "Statistics on Transfer" banners on STEP read/write.
// Clear all printers at load time per the documented recommendation.
// ******        Statistics on Transfer (Write)                 ******
static const int _silence_occt_default_printer = []() {
    Message::DefaultMessenger()->ChangePrinters().Clear();
    return 0;
}();

// ==================== Shape Constructors ====================

std::unique_ptr<TopoDS_Shape> make_empty() {
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    return std::make_unique<TopoDS_Shape>(compound);
}

std::unique_ptr<TopoDS_Shape> deep_copy(const TopoDS_Shape& shape) {
    BRepBuilderAPI_Copy copier(shape, true, false);
    return std::make_unique<TopoDS_Shape>(copier.Shape());
}

// ==================== STEP read post-processing ====================

// Recover Solids from a STEP-read Compound that has disjoint shells / loose
// faces (multi-color export from SolveSpace etc.). Returns the original
// compound if no orphan faces are found (= valid STEP, zero overhead).
//
// If `colorMap` is non-null, remaps its keys for faces whose TShape* changed
// during sewing (only applicable when called from the color path).
//
// See #129 for the reproducer and root-cause analysis.
//
// Design notes:
//   - has_orphans を残す理由: SewedShape().IsNull() でも判定可能だが、
//     (a) 空入力 Perform() を回避、(b) 「sewing 不要」と「sewing 失敗」を区別、
//     の 2 点で明示フラグ優位。
//   - Solid 配下の face を sewer に入れない理由: 既存 valid Solid の face は
//     TShape* preserve したい (colormap キー保持 + 既存挙動互換)。
//   - TopoDS_Iterator (immediate children) ではなく TopExp_Explorer (再帰) を
//     使う理由: STEP のツリー構造で valid Solid が深い所に埋まり、その兄弟に
//     orphan face がある混在ケース (例: Compound { sub { Solid + face×6 } })
//     を救うため。
//   - tolerance = Precision::Confusion(): 重複 EDGE_CURVE は座標完全一致なので
//     最厳設定で十分。緩めると意図しない縫合リスクが増す。
static TopoDS_Shape try_sew_orphan_faces(
    const TopoDS_Shape& compound,
    std::unordered_map<uint64_t, std::array<float, 3>>* colorMap)
{
    // 1. 既存 Solid と配下 face TShape* 集合を回収
    std::unordered_set<const TopoDS_TShape*> in_solid;
    std::vector<TopoDS_Shape> existing_solids;
    for (TopExp_Explorer sx(compound, TopAbs_SOLID); sx.More(); sx.Next()) {
        existing_solids.push_back(sx.Current());
        for (TopExp_Explorer fx(sx.Current(), TopAbs_FACE); fx.More(); fx.Next()) {
            in_solid.insert(fx.Current().TShape().get());
        }
    }

    // 2. 孤立 face を Sewing に投入
    BRepBuilderAPI_Sewing sewer(Precision::Confusion());
    bool has_orphans = false;
    std::vector<TopoDS_Shape> orphan_faces;  // color remap 用に保持
    for (TopExp_Explorer fx(compound, TopAbs_FACE); fx.More(); fx.Next()) {
        if (in_solid.count(fx.Current().TShape().get()) == 0) {
            sewer.Add(fx.Current());
            orphan_faces.push_back(fx.Current());
            has_orphans = true;
        }
    }

    // 3. 正常 STEP は素通し (= zero-overhead)
    if (!has_orphans) return compound;

    // 4. 縫合 → Shell ごとに MakeSolid → 新 compound 構築
    sewer.Perform();
    TopoDS_Shape sewn = sewer.SewedShape();

    BRep_Builder bb;
    TopoDS_Compound new_compound;
    bb.MakeCompound(new_compound);
    for (const auto& s : existing_solids) bb.Add(new_compound, s);
    for (TopExp_Explorer sx(sewn, TopAbs_SHELL); sx.More(); sx.Next()) {
        BRepBuilderAPI_MakeSolid mk(TopoDS::Shell(sx.Current()));
        if (mk.IsDone()) bb.Add(new_compound, mk.Solid());
    }

    // 5. colormap キー remap (color path のみ)
    if (colorMap) {
        for (const auto& old_face : orphan_faces) {
            uint64_t old_id = reinterpret_cast<uint64_t>(old_face.TShape().get());
            auto it = colorMap->find(old_id);
            if (it == colorMap->end()) continue;
            if (sewer.IsModified(old_face)) {
                uint64_t new_id = reinterpret_cast<uint64_t>(
                    sewer.Modified(old_face).TShape().get());
                (*colorMap)[new_id] = it->second;
            }
        }
    }

    return new_compound;
}

// ==================== Compound Decompose/Compose ====================

std::unique_ptr<std::vector<TopoDS_Shape>> decompose_by_kind(const TopoDS_Shape& shape, uint32_t kind) {
    auto result = std::make_unique<std::vector<TopoDS_Shape>>();
    TopAbs_ShapeEnum wanted;
    switch (kind) {
        case 1: wanted = TopAbs_COMPOUND;  break;
        case 2: wanted = TopAbs_COMPSOLID; break;
        case 3: wanted = TopAbs_SOLID;     break;
        case 4: wanted = TopAbs_SHELL;     break;
        case 5: wanted = TopAbs_FACE;      break;
        case 6: wanted = TopAbs_WIRE;      break;
        case 7: wanted = TopAbs_EDGE;      break;
        case 8: wanted = TopAbs_VERTEX;    break;
        default: return result;  // null / other name no topology to explore
    }
    for (TopExp_Explorer ex(shape, wanted); ex.More(); ex.Next()) {
        result->push_back(ex.Current());  // shallow handle copy
    }
    return result;
}

void compound_add(TopoDS_Shape& compound, const TopoDS_Shape& child) {
    BRep_Builder builder;
    builder.Add(compound, child);
}

// ==================== History relays ====================
// Every row of the algorithm table below reports its history through these
// two maps: `relay_from_builder` reads a builder's `Modified` / `IsDeleted`
// for each input (an untouched input is its own descendant; a deleted one is
// absent), `relay_from_copy` reads `BRepBuilderAPI_Copy` where a row copies
// its result. `Generated` is deliberately not read: history is descent between
// elements of one kind.
//
// Modified relations retain all source identities, including merges. Face and
// edge history share one representation; consumers filter by result topology.
using ShapeRelay = std::unordered_multimap<uint64_t, uint64_t>;

template <typename Builder>
static void relay_from_builder(
    Builder& builder,
    const TopoDS_Shape& src,
    ShapeRelay& relay)
{
    for (const auto kind : {TopAbs_FACE, TopAbs_EDGE}) {
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> shapes;
        TopExp::MapShapes(src, kind, shapes);
        for (int index = 1; index <= shapes.Extent(); ++index) {
            const auto& original = shapes(index);
            const auto source_id = reinterpret_cast<uint64_t>(original.TShape().get());
            if (builder.IsDeleted(original)) continue;
            const auto& modified = builder.Modified(original);
            if (modified.IsEmpty()) {
                relay.emplace(source_id, source_id);
            } else {
                for (NCollection_List<TopoDS_Shape>::Iterator it(modified); it.More(); it.Next()) {
                    relay.emplace(reinterpret_cast<uint64_t>(it.Value().TShape().get()), source_id);
                }
            }
        }
    }
}

static void relay_from_copy(
    BRepBuilderAPI_Copy& copier,
    const TopoDS_Shape& source,
    ShapeRelay& relay)
{
    for (const auto kind : {TopAbs_FACE, TopAbs_EDGE}) {
        NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> shapes;
        TopExp::MapShapes(source, kind, shapes);
        for (int index = 1; index <= shapes.Extent(); ++index) {
            const auto& original = shapes(index);
            const auto copied = copier.ModifiedShape(original);
            if (!copied.IsNull()) {
                relay.emplace(reinterpret_cast<uint64_t>(copied.TShape().get()),
                              reinterpret_cast<uint64_t>(original.TShape().get()));
            }
        }
    }
}

// ==================== Transforms (solid → solid, no history) ====================

std::unique_ptr<TopoDS_Shape> transform_translate(
    const TopoDS_Shape& shape,
    double tx, double ty, double tz)
{
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(tx, ty, tz));
    return std::make_unique<TopoDS_Shape>(shape.Moved(TopLoc_Location(trsf)));
}

std::unique_ptr<TopoDS_Shape> transform_rotate(
    const TopoDS_Shape& shape,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    double angle)
{
    try {
        gp_Trsf trsf;
        trsf.SetRotation(gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), angle);
        return std::make_unique<TopoDS_Shape>(shape.Moved(TopLoc_Location(trsf)));
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

// ==================== Shape Queries ====================

bool shape_is_null(const TopoDS_Shape& shape) {
    return shape.IsNull();
}

bool shape_is_closed(const TopoDS_Shape& shape) {
    return !shape.IsNull() && BRep_Tool::IsClosed(shape);
}

uint32_t shape_kind(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return 0;
    switch (shape.ShapeType()) {
        case TopAbs_COMPOUND:  return 1;
        case TopAbs_COMPSOLID: return 2;
        case TopAbs_SOLID:     return 3;
        case TopAbs_SHELL:     return 4;
        case TopAbs_FACE:      return 5;
        case TopAbs_WIRE:      return 6;
        case TopAbs_EDGE:      return 7;
        case TopAbs_VERTEX:    return 8;
        default:               return 9;
    }
}

// Adaptive integration: the fixed-order rule under-integrates a face whose
// surface has more knot spans than the rule has points (a loft, ~20% low).
constexpr double MASS_PROPERTY_EPS = 1.0e-6;

static GProp_GProps volume_properties(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props, MASS_PROPERTY_EPS);
    return props;
}

double shape_volume(const TopoDS_Shape& shape) {
    return volume_properties(shape).Mass();
}

double shape_surface_area(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(shape, props, MASS_PROPERTY_EPS);
    return props.Mass();
}

void shape_center_of_mass(const TopoDS_Shape& shape,
    double& x, double& y, double& z)
{
    GProp_GProps props = volume_properties(shape);
    gp_Pnt com = props.CentreOfMass();
    x = com.X(); y = com.Y(); z = com.Z();
}

void shape_inertia_tensor(const TopoDS_Shape& shape,
    double& m00, double& m01, double& m02,
    double& m10, double& m11, double& m12,
    double& m20, double& m21, double& m22)
{
    // OCCT's MatrixOfInertia() is expressed about the center of mass, but the
    // Rust-side API returns the tensor about the world origin so collections
    // can aggregate by plain matrix sum (parallel-axis theorem is already
    // folded in). Shift here with I_world = I_com + m·(|d|² I - d⊗d),
    // where d = COM vector from world origin, m = volume (uniform density).
    GProp_GProps props = volume_properties(shape);
    gp_Mat ic = props.MatrixOfInertia();
    gp_Pnt com = props.CentreOfMass();
    double mass = props.Mass();
    double dx = com.X(), dy = com.Y(), dz = com.Z();
    double d2 = dx*dx + dy*dy + dz*dz;
    m00 = ic.Value(1,1) + mass * (d2 - dx*dx);
    m11 = ic.Value(2,2) + mass * (d2 - dy*dy);
    m22 = ic.Value(3,3) + mass * (d2 - dz*dz);
    m01 = ic.Value(1,2) - mass * dx * dy;
    m02 = ic.Value(1,3) - mass * dx * dz;
    m12 = ic.Value(2,3) - mass * dy * dz;
    m10 = m01; m20 = m02; m21 = m12;
}

bool shape_contains_point(const TopoDS_Shape& shape, double x, double y, double z) {
    BRepClass3d_SolidClassifier classifier(shape, gp_Pnt(x, y, z), 1e-6);
    return classifier.State() == TopAbs_IN;
}

void shape_bounding_box(const TopoDS_Shape& shape,
    double& xmin, double& ymin, double& zmin,
    double& xmax, double& ymax, double& zmax)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
}

// ==================== Meshing ====================

MeshData mesh_shape(const TopoDS_Shape& shape, double linear, double angular, bool relative) {
    MeshData result;
    result.success = false;

    // BRepMesh_IncrementalMesh(shape, linDeflection, isRelative, angDeflection, isInParallel)
    IMeshTools_Parameters parameters;
    const double merge_tolerance = relative ? 0.0 : std::min(Precision::Confusion(), linear * 0.001);
    const double mesh_budget = linear - merge_tolerance;
    parameters.Deflection = mesh_budget;
    parameters.DeflectionInterior = mesh_budget;
    parameters.Angle = angular;
    parameters.AngleInterior = angular;
    parameters.Relative = relative;
    parameters.EnableControlSurfaceDeflectionAllSurfaces = true;
    bool within_budget = false;
    for (int attempt = 0; attempt < 6; ++attempt) {
        BRepMesh_IncrementalMesh mesher(shape, parameters);
        if (!mesher.IsDone()) return result;
        if (relative) { within_budget = true; break; }
        BRepLib::UpdateDeflection(shape);
        double measured = 0.0;
        for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
            TopLoc_Location location;
            const auto& mesh = BRep_Tool::Triangulation(TopoDS::Face(faces.Current()), location);
            if (mesh.IsNull() || !mesh->HasUVNodes()) return result;
            measured = std::max(measured, mesh->Deflection());
        }
        if (std::isfinite(measured) && measured <= mesh_budget) { within_budget = true; break; }
        parameters.Deflection *= 0.5;
        parameters.DeflectionInterior *= 0.5;
    }
    if (!within_budget) return result;

    uint32_t global_vertex_offset = 0;

    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        TopoDS_Face face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
        // Nodal normals, taken from the underlying surface (GeomLib::NormEstim at
        // each UV node) rather than averaged from the triangles, so curved faces
        // carry their exact normal. NOT Poly_Triangulation::ComputeNormals, which
        // only averages triangle normals and would throw the surface away.
        //
        // Safe on a null handle, and every other path allocates the array, so the
        // guard below rejects exactly the faces with nothing to emit: no
        // triangulation at all, or a triangulation with no nodes.
        BRepLib_ToolTriangulatedShape::ComputeNormals(face, triangulation);
        if (triangulation.IsNull() || !triangulation->HasNormals()) {
            return result;
        }

        // Consolidate singular surface nodes and discard collapsed facets using
        // OCCT's mesh algorithm. Reserve its displacement in the chord budget;
        // the analytic face and its stored triangulation remain untouched. The
        // merge keeps neither UV nodes nor normals, so each node's surface normal
        // is carried across by the element that placed it.
        if (!relative) {
            Poly_MergeNodesTool merger(std::acos(-1.0), merge_tolerance, triangulation->NbTriangles());
            merger.SetMergeOpposite(true);
            std::vector<gp_Dir> normals;
            for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
                int corner[3];
                triangulation->Triangle(i).Get(corner[0], corner[1], corner[2]);
                for (int k = 0; k < 3; ++k) merger.ChangeElementNode(k) = triangulation->Node(corner[k]).XYZ();
                merger.PushLastTriangle();
                for (int k = 0; k < 3; ++k) {
                    const size_t merged = static_cast<size_t>(merger.ElementNodeIndex(k));
                    if (merged >= normals.size()) normals.resize(merged + 1, gp::DZ());
                    normals[merged] = triangulation->Normal(corner[k]);
                }
            }
            triangulation = merger.Result();
            triangulation->AddNormals();
            for (size_t i = 0; i < normals.size(); ++i) triangulation->SetNormal(static_cast<int>(i) + 1, normals[i]);
        }

        int nb_nodes = triangulation->NbNodes();
        int nb_triangles = triangulation->NbTriangles();
        if (nb_nodes <= 0 || nb_triangles <= 0 ||
            static_cast<uint64_t>(global_vertex_offset) + static_cast<uint64_t>(nb_nodes) >
                std::numeric_limits<uint32_t>::max()) return result;

        // Shared by the nodal normals and the index winding below.
        bool reversed = (face.Orientation() == TopAbs_REVERSED);

        // Position and normal of every node in one pass.
        for (int i = 1; i <= nb_nodes; i++) {
            gp_Pnt p = triangulation->Node(i);
            p.Transform(location.Transformation());
            result.vertices.push_back(p.X());
            result.vertices.push_back(p.Y());
            result.vertices.push_back(p.Z());

            // ComputeNormals ignores face orientation and works in the
            // triangulation's local frame, so apply the location and the REVERSED
            // flip here — the same rule the index winding below uses.
            gp_Dir n = triangulation->Normal(i);
            n.Transform(location.Transformation());
            if (reversed) n.Reverse();
            result.normals.push_back(n.X());
            result.normals.push_back(n.Y());
            result.normals.push_back(n.Z());
        }

        // Indices
        uint64_t face_id = reinterpret_cast<uint64_t>(face.TShape().get());
        for (int i = 1; i <= nb_triangles; i++) {
            const Poly_Triangle& tri = triangulation->Triangle(i);

            int n1, n2, n3;
            tri.Get(n1, n2, n3);

            // OCC indices are 1-based, convert to 0-based + global offset
            if (reversed) {
                result.indices.push_back(global_vertex_offset + n1 - 1);
                result.indices.push_back(global_vertex_offset + n3 - 1);
                result.indices.push_back(global_vertex_offset + n2 - 1);
            } else {
                result.indices.push_back(global_vertex_offset + n1 - 1);
                result.indices.push_back(global_vertex_offset + n2 - 1);
                result.indices.push_back(global_vertex_offset + n3 - 1);
            }
            result.face_tshape_ids.push_back(face_id);
        }

        global_vertex_offset += nb_nodes;
    }

    result.success = true;
    return result;
}

// ==================== Topology enumeration ====================

std::unique_ptr<std::vector<TopoDS_Edge>> shape_edges(const TopoDS_Shape& shape) {
    // TopExp_Explorer visits shared edges once per adjacent face.
    // NCollection_IndexedMap collapses those into unique edges.
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edgeMap;
    TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
    auto out = std::make_unique<std::vector<TopoDS_Edge>>();
    out->reserve(edgeMap.Extent());
    for (int i = 1; i <= edgeMap.Extent(); i++) {
        out->push_back(TopoDS::Edge(edgeMap(i)));
    }
    return out;
}

std::unique_ptr<std::vector<TopoDS_Face>> shape_faces(const TopoDS_Shape& shape) {
    // Faces in a valid shape are already unique under TopExp_Explorer.
    auto out = std::make_unique<std::vector<TopoDS_Face>>();
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        out->push_back(TopoDS::Face(ex.Current()));
    }
    return out;
}

std::unique_ptr<std::vector<TopoDS_Edge>> face_edges(const TopoDS_Face& face) {
    // A face's outer wire and (optional) inner wires can share edges; collapse
    // them into unique edges with the same IndexedMap trick used in shape_edges.
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edgeMap;
    TopExp::MapShapes(face, TopAbs_EDGE, edgeMap);
    auto out = std::make_unique<std::vector<TopoDS_Edge>>();
    out->reserve(edgeMap.Extent());
    for (int i = 1; i <= edgeMap.Extent(); i++) {
        out->push_back(TopoDS::Edge(edgeMap(i)));
    }
    return out;
}

std::unique_ptr<TopoDS_Shape> clone_shape_handle(const TopoDS_Shape& shape) {
    return std::make_unique<TopoDS_Shape>(shape);
}

std::unique_ptr<TopoDS_Edge> clone_edge_handle(const TopoDS_Edge& edge) {
    return std::make_unique<TopoDS_Edge>(edge);
}

std::unique_ptr<TopoDS_Face> clone_face_handle(const TopoDS_Face& face) {
    return std::make_unique<TopoDS_Face>(face);
}

// ==================== Face Methods ====================

uint64_t face_tshape_id(const TopoDS_Face& face) {
    return reinterpret_cast<uint64_t>(face.TShape().get());
}

uint64_t shape_tshape_id(const TopoDS_Shape& shape) {
    return reinterpret_cast<uint64_t>(shape.TShape().get());
}

uint64_t edge_tshape_id(const TopoDS_Edge& edge) {
    return reinterpret_cast<uint64_t>(edge.TShape().get());
}

bool face_project_point(const TopoDS_Face& face,
    double px, double py, double pz,
    double& cpx, double& cpy, double& cpz,
    double& nx, double& ny, double& nz)
{
    // Default normal = zero. Returned when BRepLProp can't define a normal
    // at the closest hit (e.g. degenerate surface point or zero first
    // derivative). Caller can detect via `normal.length() == 0`.
    nx = 0.0; ny = 0.0; nz = 0.0;

    try {
        // BRepExtrema_ExtPF respects face trim, unlike Extrema_ExtPS which
        // works on the underlying infinite surface. The vertex wrapping
        // overhead (Handle alloc) is bounded — single Handle per call.
        TopoDS_Vertex vert = BRepBuilderAPI_MakeVertex(gp_Pnt(px, py, pz));
        BRepExtrema_ExtPF ext(vert, face);
        if (!ext.IsDone() || ext.NbExt() < 1) return false;

        // Pick the smallest-distance extremum.
        int best = 1;
        double best_d2 = ext.SquareDistance(1);
        for (int i = 2; i <= ext.NbExt(); ++i) {
            double d2 = ext.SquareDistance(i);
            if (d2 < best_d2) {
                best_d2 = d2;
                best = i;
            }
        }

        gp_Pnt cp = ext.Point(best);
        cpx = cp.X();
        cpy = cp.Y();
        cpz = cp.Z();

        double u, v;
        ext.Parameter(best, u, v);

        BRepAdaptor_Surface surf(face);
        BRepLProp_SLProps props(surf, u, v, /*derivOrder=*/1, Precision::Confusion());
        if (!props.IsNormalDefined()) return true;  // cp valid, normal stays 0.

        gp_Dir n = props.Normal();
        // BRepLProp returns the surface-orientation normal; flip when the
        // face is REVERSED in its enclosing shell so the caller always
        // sees an outward-pointing direction.
        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
        nx = n.X();
        ny = n.Y();
        nz = n.Z();
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

// ==================== Edge Methods ====================

rust::Vec<double> edge_approximation_segments(
    const TopoDS_Edge& edge, double linear, double angular, bool relative)
{
    rust::Vec<double> out;
    try {
        // Mirror mesh_shape's relative semantics: when relative, scale the chord
        // by the edge's bounding-box max dimension (OCCT BRepMesh convention).
        double eff_chord = linear;
        if (relative) {
            Bnd_Box box;
            BRepBndLib::Add(edge, box);
            if (!box.IsVoid()) {
                double xmin, ymin, zmin, xmax, ymax, zmax;
                box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                eff_chord = linear * std::max(xmax - xmin, std::max(ymax - ymin, zmax - zmin));
            }
        }
        BRepAdaptor_Curve curve(edge);
        GCPnts_TangentialDeflection approx(curve, angular, eff_chord);

        int nb_points = approx.NbPoints();
        for (int i = 1; i <= nb_points; i++) {
            gp_Pnt p = approx.Value(i);
            out.push_back(p.X());
            out.push_back(p.Y());
            out.push_back(p.Z());
        }
    } catch (const Standard_Failure&) {
        out.clear();
    }
    return out;
}

std::unique_ptr<TopoDS_Edge> make_helix_edge(
    double ax, double ay, double az,
    double xrx, double xry, double xrz,
    double radius, double pitch, double height)
{
    try {
        if (radius < Precision::Confusion()) return nullptr;
        if (pitch < Precision::Confusion()) return nullptr;
        if (height < Precision::Confusion()) return nullptr;

        // Build a deterministic local frame: the cylinder's local +X is the
        // user-supplied x_ref (orthogonalized against axis by gp_Ax2). The
        // helix then starts at (radius, 0, 0) in this frame, which is
        // origin + radius * normalize(x_ref ⊥ axis) in world coordinates.
        gp_Dir axis_dir(ax, ay, az);
        gp_Dir x_ref(xrx, xry, xrz);
        if (axis_dir.IsParallel(x_ref, Precision::Angular())) return nullptr;
        gp_Ax2 ax2(gp_Pnt(0.0, 0.0, 0.0), axis_dir, x_ref);
        Handle(Geom_CylindricalSurface) cylinder =
            new Geom_CylindricalSurface(ax2, radius);

        double turns = height / pitch;
        double total_angle = turns * 2.0 * M_PI;
        gp_Pnt2d line_origin(0.0, 0.0);
        gp_Dir2d line_dir(total_angle, height);
        Handle(Geom2d_Line) line2d = new Geom2d_Line(line_origin, line_dir);

        double param_end = std::sqrt(total_angle * total_angle + height * height);

        BRepBuilderAPI_MakeEdge edgeMaker(line2d, cylinder, 0.0, param_end);
        if (!edgeMaker.IsDone()) return nullptr;
        TopoDS_Edge edge = edgeMaker.Edge();
        BRepLib::BuildCurve3d(edge);
        return std::make_unique<TopoDS_Edge>(edge);
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<std::vector<TopoDS_Edge>> make_polygon_edges(rust::Slice<const double> coords) {
    auto out = std::make_unique<std::vector<TopoDS_Edge>>();
    if (coords.size() < 9 || coords.size() % 3 != 0) return out;
    try {
        BRepBuilderAPI_MakePolygon poly;
        for (size_t i = 0; i + 2 < coords.size(); i += 3) {
            poly.Add(gp_Pnt(coords[i], coords[i + 1], coords[i + 2]));
        }
        poly.Close();
        if (!poly.IsDone()) return out;
        TopoDS_Wire wire = poly.Wire();
        // Walk the wire's edges in order using TopExp_Explorer.
        for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
            out->push_back(TopoDS::Edge(ex.Current()));
        }
        return out;
    } catch (const Standard_Failure&) {
        out->clear();
        return out;
    }
}

std::unique_ptr<TopoDS_Edge> make_circle_edge(
    double ax, double ay, double az, double radius)
{
    try {
        if (radius < Precision::Confusion()) return nullptr;
        gp_Dir axis_dir(ax, ay, az);
        // Single-arg gp_Ax2(origin, N): OCCT picks an arbitrary X direction
        // orthogonal to the normal. The circle's parametric start is then at
        // (radius, 0, 0) in that implicit local frame. Callers that need a
        // specific start direction should rotate the result into place.
        gp_Ax2 ax2(gp_Pnt(0.0, 0.0, 0.0), axis_dir);
        gp_Circ circ(ax2, radius);
        BRepBuilderAPI_MakeEdge edgeMaker(circ);
        if (!edgeMaker.IsDone()) return nullptr;
        return std::make_unique<TopoDS_Edge>(edgeMaker.Edge());
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<TopoDS_Edge> make_line_edge(
    double ax, double ay, double az,
    double bx, double by, double bz)
{
    try {
        gp_Pnt a(ax, ay, az);
        gp_Pnt b(bx, by, bz);
        if (a.Distance(b) < Precision::Confusion()) return nullptr;
        BRepBuilderAPI_MakeEdge edgeMaker(a, b);
        if (!edgeMaker.IsDone()) return nullptr;
        return std::make_unique<TopoDS_Edge>(edgeMaker.Edge());
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<TopoDS_Edge> make_arc_edge(
    double sx, double sy, double sz,
    double mx, double my, double mz,
    double ex, double ey, double ez)
{
    try {
        gp_Pnt p_start(sx, sy, sz);
        gp_Pnt p_mid(mx, my, mz);
        gp_Pnt p_end(ex, ey, ez);
        // false: do not wrap around; the arc goes from start through
        // mid to end on the unique circle defined by those three points.
        GC_MakeArcOfCircle maker(p_start, p_mid, p_end);
        if (!maker.IsDone()) return nullptr;
        BRepBuilderAPI_MakeEdge edgeMaker(maker.Value());
        if (!edgeMaker.IsDone()) return nullptr;
        return std::make_unique<TopoDS_Edge>(edgeMaker.Edge());
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

// Cubic B-spline edge interpolating the given data points.
//
// `coords` is a flat array of xyz triples (length must be a multiple of 3
// and ≥ 6). Each (x, y, z) triple is one interpolation target — the
// resulting curve passes through every input point.
//
// `end_kind` selects the end-condition variant of `BSplineEnd`:
//   0 = Periodic — wraps around with C² continuity. Periodic is encoded
//       in the basis; the caller must NOT duplicate the first point at
//       the end. Needs ≥ 3 points (Rust side validates).
//   1 = NotAKnot — open curve, OCCT default end condition (the boundary
//       cubic is fit to 3 data points instead of being constrained by
//       an artificial derivative). Needs ≥ 2 points.
//   2 = Clamped — open curve with explicit start/end tangent vectors
//       passed in (sx, sy, sz) and (ex, ey, ez). Needs ≥ 2 points.
//
// For end_kind 0 and 1, the tangent arguments are ignored.
//
// Returns null on any failure (out-of-range end_kind, OCCT internal
// failure, degenerate point distribution).
std::unique_ptr<TopoDS_Edge> make_bspline_edge(
    rust::Slice<const double> coords,
    uint32_t end_kind,
    double sx, double sy, double sz,
    double ex, double ey, double ez, double tolerance)
{
    if (coords.size() < 6 || coords.size() % 3 != 0 ||
        coords.size() / 3 > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        !std::isfinite(tolerance) || tolerance <= 0) return nullptr;
    try {
        // Local alias: `Handle(NCollection_HArray1<gp_Pnt>)` は Handle マクロが
        // template 内のカンマで引数を分割してしまうので、using alias を噛ませて
        // 単一トークン化する(コミット a72e330 で deprecated 型に戻した時の回避策)。
        using HPntArray = NCollection_HArray1<gp_Pnt>;
        const int n = static_cast<int>(coords.size() / 3);
        Handle(HPntArray) pts = new HPntArray(1, n);
        for (int i = 0; i < n; ++i) {
            pts->SetValue(i + 1, gp_Pnt(coords[i * 3], coords[i * 3 + 1], coords[i * 3 + 2]));
        }

        const bool periodic = (end_kind == 0) ? true : false;
        GeomAPI_Interpolate interp(pts, periodic, tolerance);

        if (end_kind == 2) {
            // Clamped: load explicit start and end tangent vectors.
            // Scale = true lets OCCT scale the tangent magnitude
            // by the chord length, which usually gives more intuitive
            // pull strength than the raw vector magnitude.
            gp_Vec start_tan(sx, sy, sz);
            gp_Vec end_tan(ex, ey, ez);
            interp.Load(start_tan, end_tan, true);
        } else if (end_kind != 0 && end_kind != 1) {
            // Unknown end_kind; fail rather than silently picking a default.
            return nullptr;
        }

        interp.Perform();
        if (!interp.IsDone()) return nullptr;

        Handle(Geom_BSplineCurve) curve = interp.Curve();
        if (curve.IsNull()) return nullptr;

        BRepBuilderAPI_MakeEdge edgeMaker(curve);
        if (!edgeMaker.IsDone()) return nullptr;
        return std::make_unique<TopoDS_Edge>(edgeMaker.Edge());
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

void edge_endpoints(const TopoDS_Edge& edge,
    double& sx, double& sy, double& sz,
    double& ex, double& ey, double& ez)
{
    sx = 0.0; sy = 0.0; sz = 0.0;
    ex = 0.0; ey = 0.0; ez = 0.0;
    try {
        BRepAdaptor_Curve curve(edge);
        gp_Pnt start = curve.Value(curve.FirstParameter());
        gp_Pnt end = curve.Value(curve.LastParameter());
        sx = start.X(); sy = start.Y(); sz = start.Z();
        ex = end.X();   ey = end.Y();   ez = end.Z();
    } catch (const Standard_Failure&) {}
}

void edge_tangents(const TopoDS_Edge& edge,
    double& sx, double& sy, double& sz,
    double& ex, double& ey, double& ez)
{
    sx = 0.0; sy = 0.0; sz = 0.0;
    ex = 0.0; ey = 0.0; ez = 0.0;
    try {
        BRepAdaptor_Curve curve(edge);
        gp_Pnt p;
        gp_Vec vs, ve;
        curve.D1(curve.FirstParameter(), p, vs);
        curve.D1(curve.LastParameter(), p, ve);
        if (vs.Magnitude() > Precision::Confusion()) {
            vs.Normalize();
            sx = vs.X(); sy = vs.Y(); sz = vs.Z();
        }
        if (ve.Magnitude() > Precision::Confusion()) {
            ve.Normalize();
            ex = ve.X(); ey = ve.Y(); ez = ve.Z();
        }
    } catch (const Standard_Failure&) {}
}

bool edge_is_closed(const TopoDS_Edge& edge) {
    try {
        BRepAdaptor_Curve curve(edge);
        gp_Pnt p_start = curve.Value(curve.FirstParameter());
        gp_Pnt p_end   = curve.Value(curve.LastParameter());
        return p_start.Distance(p_end) < Precision::Confusion();
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool edge_project_point(const TopoDS_Edge& edge,
    double px, double py, double pz,
    double& cpx, double& cpy, double& cpz,
    double& tx, double& ty, double& tz)
{
    cpx = 0.0; cpy = 0.0; cpz = 0.0;
    tx = 0.0; ty = 0.0; tz = 0.0;
    try {
        double first = 0.0, last = 0.0;
        Handle(Geom_Curve) gcurve = BRep_Tool::Curve(edge, first, last);
        if (gcurve.IsNull()) return false;
        gp_Pnt target(px, py, pz);
        GeomAPI_ProjectPointOnCurve projector(target, gcurve, first, last);
        double u;
        if (projector.NbPoints() > 0) {
            u = projector.LowerDistanceParameter();
        } else {
            // No interior extremum within [first, last] — distance is monotonic
            // along the curve segment (e.g. line segment with target beyond an
            // endpoint). Clamp to whichever endpoint is closer.
            double d_first = target.Distance(gcurve->Value(first));
            double d_last  = target.Distance(gcurve->Value(last));
            u = (d_first <= d_last) ? first : last;
        }
        gp_Pnt p;
        gp_Vec v;
        gcurve->D1(u, p, v);
        cpx = p.X(); cpy = p.Y(); cpz = p.Z();
        if (v.Magnitude() > Precision::Confusion()) {
            v.Normalize();
            tx = v.X(); ty = v.Y(); tz = v.Z();
        }
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

std::unique_ptr<TopoDS_Edge> deep_copy_edge(const TopoDS_Edge& edge) {
    try {
        BRepBuilderAPI_Copy copier(edge);
        return std::make_unique<TopoDS_Edge>(TopoDS::Edge(copier.Shape()));
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

// Helper: apply a gp_Trsf to an edge via BRepBuilderAPI_Transform.
// Used for all four edge transforms below.
static std::unique_ptr<TopoDS_Edge> transform_edge_impl(
    const TopoDS_Edge& edge, const gp_Trsf& trsf)
{
    try {
        BRepBuilderAPI_Transform transform(edge, trsf, true);
        return std::make_unique<TopoDS_Edge>(TopoDS::Edge(transform.Shape()));
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<TopoDS_Edge> translate_edge(
    const TopoDS_Edge& edge, double tx, double ty, double tz)
{
    gp_Trsf trsf;
    trsf.SetTranslation(gp_Vec(tx, ty, tz));
    return transform_edge_impl(edge, trsf);
}

std::unique_ptr<TopoDS_Edge> rotate_edge(
    const TopoDS_Edge& edge,
    double ox, double oy, double oz,
    double dx, double dy, double dz,
    double angle)
{
    try {
        gp_Trsf trsf;
        trsf.SetRotation(gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), angle);
        return transform_edge_impl(edge, trsf);
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<TopoDS_Edge> scale_edge(
    const TopoDS_Edge& edge,
    double cx, double cy, double cz,
    double factor)
{
    gp_Trsf trsf;
    trsf.SetScale(gp_Pnt(cx, cy, cz), factor);
    return transform_edge_impl(edge, trsf);
}

std::unique_ptr<TopoDS_Edge> mirror_edge(
    const TopoDS_Edge& edge,
    double ox, double oy, double oz,
    double nx, double ny, double nz)
{
    try {
        gp_Trsf trsf;
        trsf.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz)));
        return transform_edge_impl(edge, trsf);
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

std::unique_ptr<std::vector<TopoDS_Shape>> shape_vec_new() {
    return std::make_unique<std::vector<TopoDS_Shape>>();
}

void shape_vec_push(std::vector<TopoDS_Shape>& v, const TopoDS_Shape& s) {
    v.push_back(s);
}

void shape_vec_push_edge(std::vector<TopoDS_Shape>& v, const TopoDS_Edge& e) {
    v.push_back(e);
}

void shape_vec_push_face(std::vector<TopoDS_Shape>& v, const TopoDS_Face& f) {
    v.push_back(f);
}

static GeomAbs_JoinType join_type(uint32_t join, const std::string& where) {
    switch (join) {
        case 0: return GeomAbs_Arc;
        case 1: return GeomAbs_Tangent;
        case 2: return GeomAbs_Intersection;
        default: throw std::runtime_error(where + ": unknown join type " + std::to_string(join));
    }
}

static GeomAbs_Shape continuity_order(uint32_t continuity, const std::string& where) {
    switch (continuity) {
        case 0: return GeomAbs_C0;
        case 1: return GeomAbs_G1;
        case 2: return GeomAbs_G2;
        default: throw std::runtime_error(where + ": unknown continuity " + std::to_string(continuity));
    }
}

// ==================== The algorithm table ====================
//
// One entry point, `apply_algorithm`, over every OCCT algorithm the binding
// offers. A row is the constructor call and nothing else: input shapes arrive
// in one vector (selections such as fillet edges or open faces are shapes in
// that vector too), numbers in `scalars`, codes and counts in `integers`, laid
// out as `occt::algorithm::Algorithm::call` in Rust states them. Failure,
// history and the swept ends are read once, below the rows, through the
// `BRepBuilderAPI_MakeShape` contract (`Modified` / `IsDeleted`; an untouched
// input is its own descendant) that every builder in the table honours, or
// through the four-line adapters for the builders that spell it differently
// (sewing, cells, unification).
//
// Designed twice: (1) one hand-written function per algorithm, each carrying
// its own argument marshalling, try/catch, null check and history walk -- the
// state before this table, ~730 lines for fifteen algorithms, every new one a
// copy; (2) one switch over rows sharing the marshalling and the extraction.
// (2) is taken: adding an algorithm is a row here and a variant in Rust.

namespace {

// Row codes, mirrored by `Algorithm::code` in `src/occt/algorithm.rs`.
enum Row : uint32_t {
    ROW_BOX = 0,
    ROW_SPHERE = 1,
    ROW_CYLINDER = 2,
    ROW_CONE = 3,
    ROW_TORUS = 4,
    ROW_HALF_SPACE = 5,
    ROW_WIRE = 6,
    ROW_FACE = 7,
    ROW_PRISM = 8,
    ROW_REVOLUTION = 9,
    ROW_PIPE_SHELL = 10,
    ROW_THRU_SECTIONS = 11,
    ROW_OFFSET_SHAPE = 12,
    ROW_OFFSET_FACES = 13,
    ROW_THICK_SOLID = 14,
    ROW_SOLID = 15,
    ROW_FILLING = 16,
    ROW_SEW = 17,
    ROW_BOOLEAN = 18,
    ROW_SPLITTER = 19,
    ROW_SECTION = 20,
    ROW_CELLS = 21,
    ROW_FILLET = 22,
    ROW_CHAMFER = 23,
    ROW_TRANSFORM = 24,
    ROW_DRAFT_ANGLE = 25,
    ROW_PROJECTION = 26,
    ROW_UNIFY = 27,
    ROW_DEFEATURING = 28,
};

// The arguments of one call, read by position. Every accessor refuses a
// position the caller did not fill rather than reading past the slice.
struct Call {
    const std::vector<TopoDS_Shape>& shapes;
    rust::Slice<const double> scalars;
    rust::Slice<const int64_t> integers;

    const TopoDS_Shape& shape(size_t index) const {
        if (index >= shapes.size()) throw std::invalid_argument("fewer input shapes than the row takes");
        return shapes[index];
    }
    double scalar(size_t index) const {
        if (index >= scalars.size()) throw std::invalid_argument("fewer scalars than the row takes");
        return scalars[index];
    }
    int64_t integer(size_t index) const {
        if (index >= integers.size()) throw std::invalid_argument("fewer integers than the row takes");
        return integers[index];
    }
    size_t count(size_t index) const {
        const int64_t value = integer(index);
        if (value < 0) throw std::invalid_argument("a count is negative");
        return static_cast<size_t>(value);
    }
    gp_Pnt pnt(size_t at) const { return gp_Pnt(scalar(at), scalar(at + 1), scalar(at + 2)); }
    gp_Vec vec(size_t at) const { return gp_Vec(scalar(at), scalar(at + 1), scalar(at + 2)); }
    gp_Dir dir(size_t at) const { return gp_Dir(vec(at)); }
    NCollection_List<TopoDS_Shape> list(size_t from, size_t to) const {
        NCollection_List<TopoDS_Shape> shapes;
        for (size_t index = from; index < to; ++index) shapes.Append(shape(index));
        return shapes;
    }
};

struct Product {
    TopoDS_Shape shape;
    ShapeRelay relay;
    std::vector<TopoDS_Shape> ends;
};

// The two section instances a sweep or a loft places at its ends, for the
// builders that publish them.
template <class Builder>
void ends_of(Builder& builder, std::vector<TopoDS_Shape>& ends) {
    if constexpr (std::is_base_of_v<BRepPrimAPI_MakeSweep, Builder> || std::is_same_v<BRepOffsetAPI_ThruSections, Builder>) {
        ends = {builder.FirstShape(), builder.LastShape()};
    }
}

// The offset builders publish an offset face's image through `Generated` and
// reserve `Modified` for the closing-face case, so neither map alone carries
// their correspondence. This adapter presents their union where the relay
// reads `Modified`, leaving an untouched input its own descendant.
template <class Builder>
struct OffsetImages {
    Builder& builder;
    NCollection_List<TopoDS_Shape> images;
    bool IsDone() const { return builder.IsDone(); }
    const TopoDS_Shape& Shape() { return builder.Shape(); }
    bool IsDeleted(const TopoDS_Shape& shape) { return builder.IsDeleted(shape); }
    const NCollection_List<TopoDS_Shape>& Modified(const TopoDS_Shape& shape) {
        images.Clear();
        for (NCollection_List<TopoDS_Shape>::Iterator kept(builder.Modified(shape)); kept.More(); kept.Next()) images.Append(kept.Value());
        if (images.IsEmpty()) images.Append(shape);
        for (NCollection_List<TopoDS_Shape>::Iterator made(builder.Generated(shape)); made.More(); made.Next()) images.Append(made.Value());
        return images;
    }
};

// What every row returns: the built shape, checked, with the history of every
// input read through the builder.
template <class Builder>
Product done(Builder& builder, const Call& call) {
    if (!builder.IsDone()) throw std::runtime_error("did not complete");
    Product product{builder.Shape(), {}, {}};
    if (product.shape.IsNull()) throw std::runtime_error("returned an empty shape");
    for (const auto& input : call.shapes) relay_from_builder(builder, input, product.relay);
    ends_of(builder, product.ends);
    return product;
}

template <class Builder>
Product built(Builder& builder, const Call& call) {
    builder.Build();
    return done(builder, call);
}

template <class Builder>
Product boolean_row(Builder& builder, const Call& call, size_t arguments) {
    builder.SetArguments(call.list(0, arguments));
    builder.SetTools(call.list(arguments, call.shapes.size()));
    builder.Build();
    if (builder.HasErrors()) throw std::runtime_error("reported errors");
    return done(builder, call);
}

Product row(Row row, const Call& call) {
    switch (row) {
        case ROW_BOX: {
            BRepPrimAPI_MakeBox builder(call.pnt(0), call.pnt(3));
            return built(builder, call);
        }
        case ROW_SPHERE: {
            BRepPrimAPI_MakeSphere builder(call.pnt(0), call.scalar(3));
            return built(builder, call);
        }
        case ROW_CYLINDER: {
            BRepPrimAPI_MakeCylinder builder(gp_Ax2(call.pnt(0), call.dir(3)), call.scalar(6), call.scalar(7));
            return built(builder, call);
        }
        case ROW_CONE: {
            BRepPrimAPI_MakeCone builder(gp_Ax2(call.pnt(0), call.dir(3)), call.scalar(6), call.scalar(7), call.scalar(8));
            return built(builder, call);
        }
        case ROW_TORUS: {
            BRepPrimAPI_MakeTorus builder(gp_Ax2(call.pnt(0), call.dir(3)), call.scalar(6), call.scalar(7));
            return built(builder, call);
        }
        case ROW_HALF_SPACE: {
            // The material lies on the side the normal points to.
            const gp_Pnt origin = call.pnt(0);
            const gp_Dir normal = call.dir(3);
            BRepBuilderAPI_MakeFace face(gp_Pln(origin, normal));
            BRepPrimAPI_MakeHalfSpace builder(face.Face(), origin.Translated(gp_Vec(normal)));
            return built(builder, call);
        }
        case ROW_WIRE: {
            BRepBuilderAPI_MakeWire builder;
            for (const auto& edge : call.shapes) builder.Add(TopoDS::Edge(edge));
            return built(builder, call);
        }
        case ROW_FACE: {
            BRepBuilderAPI_MakeFace builder(TopoDS::Wire(call.shape(0)));
            return built(builder, call);
        }
        case ROW_PRISM: {
            BRepPrimAPI_MakePrism builder(call.shape(0), call.vec(0));
            return built(builder, call);
        }
        case ROW_REVOLUTION: {
            BRepPrimAPI_MakeRevol builder(call.shape(0), gp_Ax1(call.pnt(0), call.dir(3)), call.scalar(6));
            return built(builder, call);
        }
        case ROW_PIPE_SHELL: {
            // integers: [frame, has auxiliary spine, sections, law samples, solid]
            // scalars: [up xyz, tolerance (NaN: the builder's own), stations..., scales...]
            BRepOffsetAPI_MakePipeShell builder(TopoDS::Wire(call.shape(0)));
            size_t next = 1;
            switch (call.integer(0)) {
                case 0: {
                    BRepAdaptor_Curve curve(TopoDS::Edge(TopExp_Explorer(call.shape(0), TopAbs_EDGE).Current()));
                    gp_Pnt start;
                    gp_Vec tangent;
                    curve.D1(curve.FirstParameter(), start, tangent);
                    const gp_Dir along(tangent);
                    builder.SetMode(gp_Ax2(start, along, std::abs(along.X()) < 0.9 ? gp_Dir(1, 0, 0) : gp_Dir(0, 1, 0)));
                    break;
                }
                case 1: builder.SetMode(true); break;
                case 2: builder.SetMode(call.dir(0)); break;
                case 3: builder.SetMode(TopoDS::Wire(call.shape(next++)), true); break;
                case 4: builder.SetMode(false); break;
                default: throw std::invalid_argument("unknown sweep frame");
            }
            const size_t sections = call.count(2);
            const size_t samples = call.count(3);
            if (samples > 0) {
                if (sections != 1) throw std::invalid_argument("a scale law sweeps exactly one section");
                NCollection_Array1<gp_Pnt2d> values(1, static_cast<int>(samples));
                for (size_t sample = 0; sample < samples; ++sample) {
                    values.SetValue(static_cast<int>(sample + 1), gp_Pnt2d(call.scalar(4 + sample), call.scalar(4 + samples + sample)));
                }
                Handle(Law_Interpol) law = new Law_Interpol();
                law->Set(values, false);
                builder.SetLaw(TopoDS::Wire(call.shape(next)), law, false, true);
            } else {
                for (size_t section = 0; section < sections; ++section) builder.Add(TopoDS::Wire(call.shape(next + section)), false, false);
            }
            if (std::isfinite(call.scalar(3))) builder.SetTolerance(call.scalar(3), call.scalar(3), call.scalar(3));
            builder.Build();
            if (call.integer(4) != 0 && (!builder.IsDone() || !builder.MakeSolid())) throw std::runtime_error("could not close the swept shell into a solid");
            return done(builder, call);
        }
        case ROW_THRU_SECTIONS: {
            BRepOffsetAPI_ThruSections builder(call.integer(1) != 0, call.integer(0) != 0, call.scalar(0));
            for (const auto& section : call.shapes) builder.AddWire(TopoDS::Wire(section));
            return built(builder, call);
        }
        case ROW_OFFSET_SHAPE: {
            BRepOffsetAPI_MakeOffsetShape builder;
            builder.PerformByJoin(call.shape(0), call.scalar(0), call.scalar(1), BRepOffset_Skin, call.integer(1) != 0, false, join_type(static_cast<uint32_t>(call.integer(0)), "offset"), false);
            OffsetImages<BRepOffsetAPI_MakeOffsetShape> images{builder, {}};
            return done(images, call);
        }
        case ROW_OFFSET_FACES: {
            BRepOffset_MakeOffset builder;
            builder.Initialize(call.shape(0), 0.0, call.scalar(1), BRepOffset_Skin, true, false, GeomAbs_Intersection);
            for (size_t face = 1; face < call.shapes.size(); ++face) builder.SetOffsetOnFace(TopoDS::Face(call.shape(face)), call.scalar(0));
            builder.MakeOffsetShape();
            return done(builder, call);
        }
        case ROW_THICK_SOLID: {
            BRepOffsetAPI_MakeThickSolid builder;
            builder.MakeThickSolidByJoin(call.shape(0), call.list(1, call.shapes.size()), call.scalar(0), call.scalar(1), BRepOffset_Skin, false, false, join_type(static_cast<uint32_t>(call.integer(0)), "thicken"));
            builder.Build();
            OffsetImages<BRepOffsetAPI_MakeThickSolid> images{builder, {}};
            Product product = done(images, call);
            // The builder does not flag the removed faces as deleted; nothing in
            // the result descends from them.
            for (size_t face = 1; face < call.shapes.size(); ++face) {
                const auto removed = reinterpret_cast<uint64_t>(call.shape(face).TShape().get());
                for (auto pair = product.relay.begin(); pair != product.relay.end();) {
                    pair = pair->second == removed ? product.relay.erase(pair) : std::next(pair);
                }
            }
            return product;
        }
        case ROW_SOLID: {
            // The first shell bounds the material; every further shell is a
            // cavity, reversed so its normals face the void.
            BRepBuilderAPI_MakeSolid builder(TopoDS::Shell(call.shape(0)));
            for (size_t cavity = 1; cavity < call.shapes.size(); ++cavity) builder.Add(TopoDS::Shell(call.shape(cavity).Reversed()));
            Product product = built(builder, call);
            TopoDS_Solid solid = TopoDS::Solid(product.shape);
            BRepLib::OrientClosedSolid(solid);
            product.shape = solid;
            return product;
        }
        case ROW_FILLING: {
            // integers: [continuity, degree, points on curve, iterations, max degree, max segments]
            // scalars: [tolerance 2d, tolerance 3d, tolerance angular, tolerance curvature]
            BRepOffsetAPI_MakeFilling builder(static_cast<int>(call.integer(1)), static_cast<int>(call.integer(2)), static_cast<int>(call.integer(3)), false, call.scalar(0), call.scalar(1), call.scalar(2), call.scalar(3), static_cast<int>(call.integer(4)), static_cast<int>(call.integer(5)));
            const GeomAbs_Shape order = continuity_order(static_cast<uint32_t>(call.integer(0)), "fill");
            for (const auto& edge : call.shapes) builder.Add(TopoDS::Edge(edge), order, true);
            return built(builder, call);
        }
        case ROW_SEW: {
            BRepBuilderAPI_Sewing sewing(call.scalar(0));
            for (const auto& face : call.shapes) sewing.Add(face);
            sewing.Perform();
            Product product{sewing.SewedShape(), {}, {}};
            if (product.shape.IsNull()) throw std::runtime_error("returned an empty shape");
            for (const auto& face : call.shapes) {
                const auto source = reinterpret_cast<uint64_t>(face.TShape().get());
                const TopoDS_Shape& sewn = sewing.IsModified(face) ? sewing.Modified(face) : face;
                product.relay.emplace(reinterpret_cast<uint64_t>(sewn.TShape().get()), source);
            }
            return product;
        }
        case ROW_BOOLEAN: {
            BRepAlgoAPI_BooleanOperation builder;
            switch (call.integer(0)) {
                case 0: builder.SetOperation(BOPAlgo_FUSE); break;
                case 1: builder.SetOperation(BOPAlgo_CUT); break;
                case 2: builder.SetOperation(BOPAlgo_COMMON); break;
                default: throw std::invalid_argument("unknown boolean operation");
            }
            return boolean_row(builder, call, call.count(1));
        }
        case ROW_SPLITTER: {
            BRepAlgoAPI_Splitter builder;
            return boolean_row(builder, call, call.count(0));
        }
        case ROW_SECTION: {
            BRepAlgoAPI_Section builder;
            return boolean_row(builder, call, call.count(0));
        }
        case ROW_CELLS: {
            // integers: the DIMACS-flat DNF (`+i` takes shape i-1, `-i` avoids
            // it, `0` ends a clause). A lone shape taken whole is copied, which
            // is what the cells builder would return for it.
            if (call.shapes.size() == 1 && call.integers.size() == 2 && call.integer(0) == 1 && call.integer(1) == 0) {
                BRepBuilderAPI_Copy copier(call.shape(0), true, false);
                Product product{copier.Shape(), {}, {}};
                relay_from_copy(copier, call.shape(0), product.relay);
                return product;
            }
            BOPAlgo_CellsBuilder cells;
            cells.SetArguments(call.list(0, call.shapes.size()));
            cells.Perform();
            if (cells.HasErrors()) throw std::runtime_error("reported errors");
            NCollection_List<TopoDS_Shape> take, avoid;
            for (const int64_t literal : call.integers) {
                if (literal == 0) {
                    if (!take.IsEmpty()) cells.AddToResult(take, avoid, 1);
                    take.Clear();
                    avoid.Clear();
                    continue;
                }
                const TopoDS_Shape& shape = call.shape(static_cast<size_t>(std::llabs(literal)) - 1);
                (literal > 0 ? take : avoid).Append(shape);
            }
            cells.RemoveInternalBoundaries();
            // The result shares no geometry with its inputs: the copy severs
            // it, and its own history composes with the builder's.
            ShapeRelay built_relay;
            for (const auto& input : call.shapes) relay_from_builder(cells, input, built_relay);
            BRepBuilderAPI_Copy copier(cells.Shape(), true, false);
            ShapeRelay copied_relay;
            relay_from_copy(copier, cells.Shape(), copied_relay);
            Product product{copier.Shape(), {}, {}};
            for (const auto& pair : copied_relay) {
                const auto sources = built_relay.equal_range(pair.second);
                for (auto source = sources.first; source != sources.second; ++source) product.relay.emplace(pair.first, source->second);
            }
            return product;
        }
        case ROW_FILLET: {
            // integers: [station count]; scalars: [radius, stations..., radii...].
            // With no station the radius is constant; with stations the builder
            // interpolates them along the contour, each `(relative parameter in
            // [0,1], radius)`.
            BRepFilletAPI_MakeFillet builder(call.shape(0));
            const size_t stations = call.count(0);
            std::vector<gp_Pnt2d> law;
            law.reserve(stations);
            for (size_t station = 0; station < stations; ++station) law.emplace_back(call.scalar(1 + station), call.scalar(1 + stations + station));
            for (size_t edge = 1; edge < call.shapes.size(); ++edge) {
                const TopoDS_Edge& contour = TopoDS::Edge(call.shape(edge));
                if (law.empty()) {
                    builder.Add(call.scalar(0), contour);
                } else {
                    builder.Add(NCollection_Array1<gp_Pnt2d>(law.front(), 1, static_cast<int>(law.size())), contour);
                }
            }
            return built(builder, call);
        }
        case ROW_CHAMFER: {
            // integers: [form, edge count]; scalars: [distance] and, for the two
            // asymmetric forms, the second distance or the angle. Those forms
            // measure the distance ON a reference face, one per edge, which
            // follows the edges in the shape vector.
            BRepFilletAPI_MakeChamfer builder(call.shape(0));
            const size_t edges = call.count(1);
            for (size_t edge = 0; edge < edges; ++edge) {
                const TopoDS_Edge& contour = TopoDS::Edge(call.shape(1 + edge));
                switch (call.integer(0)) {
                    case 0: builder.Add(call.scalar(0), contour); break;
                    case 1: builder.Add(call.scalar(0), call.scalar(1), contour, TopoDS::Face(call.shape(1 + edges + edge))); break;
                    case 2: builder.AddDA(call.scalar(0), call.scalar(1), contour, TopoDS::Face(call.shape(1 + edges + edge))); break;
                    default: throw std::invalid_argument("unknown chamfer form");
                }
            }
            return built(builder, call);
        }
        case ROW_TRANSFORM: {
            // scalars: a row-major 3 by 4 affine matrix.
            gp_GTrsf transform;
            for (int line = 1; line <= 3; ++line) {
                for (int column = 1; column <= 4; ++column) transform.SetValue(line, column, call.scalar(static_cast<size_t>((line - 1) * 4 + column - 1)));
            }
            BRepBuilderAPI_GTransform builder(call.shape(0), transform, true);
            return built(builder, call);
        }
        case ROW_DRAFT_ANGLE: {
            // scalars: [direction xyz, angle, neutral plane origin xyz, neutral plane normal xyz]
            BRepOffsetAPI_DraftAngle builder(call.shape(0));
            const gp_Pln neutral(call.pnt(4), call.dir(7));
            for (size_t face = 1; face < call.shapes.size(); ++face) builder.Add(TopoDS::Face(call.shape(face)), call.dir(0), call.scalar(3), neutral);
            return built(builder, call);
        }
        case ROW_PROJECTION: {
            BRepProj_Projection builder(call.shape(0), call.shape(1), call.dir(0));
            if (!builder.IsDone()) throw std::runtime_error("did not complete");
            return Product{builder.Shape(), {}, {}};
        }
        case ROW_UNIFY: {
            ShapeUpgrade_UnifySameDomain builder(call.shape(0), true, true, true);
            builder.AllowInternalEdges(false);
            builder.Build();
            Product product{builder.Shape(), {}, {}};
            if (product.shape.IsNull()) throw std::runtime_error("returned an empty shape");
            const Handle(BRepTools_History) history = builder.History();
            if (history.IsNull()) return product;
            for (TopExp_Explorer faces(call.shape(0), TopAbs_FACE); faces.More(); faces.Next()) {
                const TopoDS_Shape& face = faces.Current();
                if (history->IsRemoved(face)) continue;
                const auto& merged = history->Modified(face);
                const TopoDS_Shape& post = merged.IsEmpty() ? face : merged.First();
                product.relay.emplace(reinterpret_cast<uint64_t>(post.TShape().get()), reinterpret_cast<uint64_t>(face.TShape().get()));
            }
            return product;
        }
        case ROW_DEFEATURING: {
            BRepAlgoAPI_Defeaturing builder;
            builder.SetShape(call.shape(0));
            builder.AddFacesToRemove(call.list(1, call.shapes.size()));
            builder.Build();
            if (builder.HasErrors()) throw std::runtime_error("reported errors");
            return done(builder, call);
        }
    }
    throw std::invalid_argument("unknown algorithm row");
}

const char* row_name(uint32_t code) {
    static const char* const names[] = {
        "box", "sphere", "cylinder", "cone", "torus", "half space", "wire", "face", "prism", "revolution",
        "pipe shell", "thru sections", "offset shape", "offset faces", "thick solid", "solid", "filling", "sew",
        "boolean", "splitter", "section", "cells", "fillet", "chamfer", "transform", "draft angle", "projection", "unify",
        "defeaturing",
    };
    return code < sizeof(names) / sizeof(names[0]) ? names[code] : "unknown";
}

}  // namespace

std::unique_ptr<TopoDS_Shape> apply_algorithm(
    uint32_t algorithm,
    const std::vector<TopoDS_Shape>& shapes,
    rust::Slice<const double> scalars,
    rust::Slice<const int64_t> integers,
    rust::Vec<uint64_t>& out_history,
    std::vector<TopoDS_Shape>& out_ends)
{
    const std::string name(row_name(algorithm));
    try {
        Product product = row(static_cast<Row>(algorithm), Call{shapes, scalars, integers});
        std::set<std::pair<uint64_t, uint64_t>> seen;
        for (const auto& pair : product.relay) {
            // Each pair once, in the order the relay yields it.
            if (!seen.insert({pair.first, pair.second}).second) continue;
            out_history.push_back(pair.first);
            out_history.push_back(pair.second);
        }
        out_ends = std::move(product.ends);
        return std::make_unique<TopoDS_Shape>(product.shape);
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(name + ": " + error.what());
    } catch (const std::exception& error) {
        throw std::runtime_error(name + ": " + error.what());
    }
}

std::unique_ptr<std::vector<TopoDS_Edge>> free_boundary_edges(
    const TopoDS_Shape& shape,
    bool split_closed,
    bool split_open,
    rust::Vec<uint32_t>& out_loop_sizes)
{
    try {
        ShapeAnalysis_FreeBounds analysis(
            shape,
            split_closed,
            split_open,
            /*checkinternaledges=*/ false);
        auto edges = std::make_unique<std::vector<TopoDS_Edge>>();
        const TopoDS_Compound loops[2] = {analysis.GetClosedWires(), analysis.GetOpenWires()};
        for (const TopoDS_Compound& wires : loops) {
            for (TopExp_Explorer wire_ex(wires, TopAbs_WIRE); wire_ex.More(); wire_ex.Next()) {
                uint32_t size = 0;
                for (TopExp_Explorer ex(wire_ex.Current(), TopAbs_EDGE); ex.More(); ex.Next()) {
                    edges->push_back(TopoDS::Edge(ex.Current()));
                    ++size;
                }
                out_loop_sizes.push_back(size);
            }
        }
        return edges;
    } catch (const Standard_Failure& error) {
        throw std::runtime_error(std::string("free boundaries: ") + error.what());
    }
}

std::unique_ptr<TopoDS_Shape> make_bspline_solid(
    rust::Slice<const double> coords,
    uint32_t nu, uint32_t nv,
    bool u_periodic)
{
    try {
        if (coords.size() != static_cast<size_t>(nu) * nv * 3) return nullptr;
        if (nu < 2 || nv < 3) return nullptr;

        // Tensor-product truly-periodic interpolation (#120).
        //
        // Naive approach (Interpolate over augmented grid → SetUPeriodic) only
        // delivers C^0 at the seam: the non-periodic interpolator picks
        // independent boundary derivatives at u_min vs u_max, and SetUPeriodic
        // just relabels topology without fixing the derivative mismatch.
        //
        // Instead we apply GeomAPI_Interpolate (which honors a true periodic
        // boundary by solving a circulant linear system) once per V column,
        // then once per U row of the resulting intermediate poles. The final
        // poles array feeds Geom_BSplineSurface(...) directly with the
        // UPeriodic / VPeriodic flags, yielding C^(degree-1) continuity at
        // both seams.
        using HPntArray  = NCollection_HArray1<gp_Pnt>;
        using HRealArray = NCollection_HArray1<double>;
        const double tol = Precision::Confusion();

        // Build uniform parameter arrays so that every column / row uses the
        // SAME parametrization. With chord-length (the Interpolate default),
        // columns of different total length get different knot vectors and
        // the resulting tensor surface has parameter mismatches at +X axis
        // (visible as boolean-intersect degeneracies). Uniform params avoid
        // this since the input grid samples φ / θ at constant fractional
        // intervals along each direction.
        const int u_param_count = static_cast<int>(u_periodic ? nu + 1 : nu);
        Handle(HRealArray) u_params = new HRealArray(1, u_param_count);
        for (int k = 0; k < u_param_count; ++k) {
            u_params->SetValue(k + 1, static_cast<double>(k) / static_cast<double>(nu));
        }
        Handle(HRealArray) v_params = new HRealArray(1, static_cast<int>(nv + 1));
        for (uint32_t k = 0; k <= nv; ++k) {
            v_params->SetValue(static_cast<int>(k) + 1,
                               static_cast<double>(k) / static_cast<double>(nv));
        }

        // Stage 1: per-V-column interpolation along U with uniform params.
        std::vector<Handle(Geom_BSplineCurve)> u_curves;
        u_curves.reserve(nv);
        for (uint32_t j = 0; j < nv; ++j) {
            Handle(HPntArray) col = new HPntArray(1, static_cast<int>(nu));
            for (uint32_t i = 0; i < nu; ++i) {
                const size_t idx = (static_cast<size_t>(i) * nv + j) * 3;
                col->SetValue(static_cast<int>(i) + 1,
                              gp_Pnt(coords[idx], coords[idx + 1], coords[idx + 2]));
            }
            GeomAPI_Interpolate interp(col, u_params, u_periodic, tol);
            interp.Perform();
            if (!interp.IsDone()) return nullptr;
            u_curves.push_back(interp.Curve());
        }

        // Capture U knot vector / multiplicities / degree from any column;
        // GeomAPI_Interpolate uses the same chord-length parametrization for
        // all columns since the V coordinate is uniform per column.
        const int u_degree = u_curves[0]->Degree();
        const int u_npoles = u_curves[0]->NbPoles();
        const NCollection_Array1<double>& u_knots = u_curves[0]->Knots();
        const NCollection_Array1<int>&    u_mults = u_curves[0]->Multiplicities();

        NCollection_Array2<gp_Pnt> intermediate(1, u_npoles, 1, static_cast<int>(nv));
        for (uint32_t j = 0; j < nv; ++j) {
            for (int i = 1; i <= u_npoles; ++i) {
                intermediate.SetValue(i, static_cast<int>(j) + 1, u_curves[j]->Pole(i));
            }
        }

        // Stage 2: per-U-row interpolation along V (V is always periodic).
        std::vector<Handle(Geom_BSplineCurve)> v_curves;
        v_curves.reserve(u_npoles);
        for (int i = 1; i <= u_npoles; ++i) {
            Handle(HPntArray) row = new HPntArray(1, static_cast<int>(nv));
            for (uint32_t j = 0; j < nv; ++j) {
                row->SetValue(static_cast<int>(j) + 1, intermediate(i, static_cast<int>(j) + 1));
            }
            GeomAPI_Interpolate interp(row, v_params, /*periodic=*/true, tol);
            interp.Perform();
            if (!interp.IsDone()) return nullptr;
            v_curves.push_back(interp.Curve());
        }

        const int v_degree = v_curves[0]->Degree();
        const int v_npoles = v_curves[0]->NbPoles();
        const NCollection_Array1<double>& v_knots = v_curves[0]->Knots();
        const NCollection_Array1<int>&    v_mults = v_curves[0]->Multiplicities();

        // Stage 3: assemble final M_pole × N_pole pole grid and build the
        // surface with explicit periodic flags.
        NCollection_Array2<gp_Pnt> final_poles(1, u_npoles, 1, v_npoles);
        for (int i = 1; i <= u_npoles; ++i) {
            for (int j = 1; j <= v_npoles; ++j) {
                final_poles.SetValue(i, j, v_curves[i - 1]->Pole(j));
            }
        }

        Handle(Geom_BSplineSurface) surface = new Geom_BSplineSurface(
            final_poles,
            u_knots, v_knots,
            u_mults, v_mults,
            u_degree, v_degree,
            /*UPeriodic=*/u_periodic,
            /*VPeriodic=*/true);
        if (surface.IsNull()) return nullptr;

        // Side face spans the full parametric domain.
        double u1, u2, v1, v2;
        surface->Bounds(u1, u2, v1, v2);
        BRepBuilderAPI_MakeFace face_maker(surface, Precision::Confusion());
        if (!face_maker.IsDone()) return nullptr;
        TopoDS_Face side_face = face_maker.Face();

        BRepBuilderAPI_Sewing sewing(1.0e-3);
        sewing.Add(side_face);

        // For non-periodic U, cap the two U-boundary loops with planar faces.
        // For periodic U the surface is already closed into a torus — no caps.
        if (!u_periodic) {
            auto make_cap = [&](double u_at) -> TopoDS_Face {
                Handle(Geom_Curve) iso = surface->UIso(u_at);
                if (iso.IsNull()) return TopoDS_Face();
                BRepBuilderAPI_MakeEdge em(iso, v1, v2);
                if (!em.IsDone()) return TopoDS_Face();
                BRepBuilderAPI_MakeWire wm(em.Edge());
                if (!wm.IsDone()) return TopoDS_Face();
                BRepBuilderAPI_MakeFace mf(wm.Wire(), true);
                return mf.IsDone() ? mf.Face() : TopoDS_Face();
            };
            TopoDS_Face cap1 = make_cap(u1);
            TopoDS_Face cap2 = make_cap(u2);
            if (cap1.IsNull() || cap2.IsNull()) return nullptr;
            sewing.Add(cap1);
            sewing.Add(cap2);
        }

        sewing.Perform();
        TopoDS_Shape sewn = sewing.SewedShape();
        if (sewn.IsNull()) return nullptr;

        TopoDS_Shell shell;
        if (sewn.ShapeType() == TopAbs_SHELL) {
            shell = TopoDS::Shell(sewn);
        } else if (sewn.ShapeType() == TopAbs_SOLID) {
            return std::make_unique<TopoDS_Shape>(sewn);
        } else if (sewn.ShapeType() == TopAbs_FACE && u_periodic) {
            // Full torus: single closed face → wrap manually.
            BRep_Builder bb;
            bb.MakeShell(shell);
            bb.Add(shell, TopoDS::Face(sewn));
            shell.Closed(true);
        } else {
            TopExp_Explorer exp(sewn, TopAbs_SHELL);
            if (exp.More()) {
                shell = TopoDS::Shell(exp.Current());
            } else {
                return nullptr;
            }
        }

        BRepBuilderAPI_MakeSolid solid_maker(shell);
        if (!solid_maker.IsDone()) return nullptr;
        TopoDS_Solid solid = solid_maker.Solid();

        // Ensure outward-facing orientation.
        BRepClass3d_SolidClassifier classifier(
            solid, gp_Pnt(0, 0, 0), Precision::Confusion());
        if (classifier.State() == TopAbs_IN) {
            solid.Reverse();
        }

        return std::make_unique<TopoDS_Shape>(solid);
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

// ==================== Streambuf bridges (ffi.cpp-internal) ====================

// std::streambuf subclass that reads from a Rust `dyn Read` via FFI callback
class RustReadStreambuf : public std::streambuf {
public:
    explicit RustReadStreambuf(RustReader& reader) : reader_(reader) {}

protected:
    int_type underflow() override {
        rust::Slice<uint8_t> slice(
            reinterpret_cast<uint8_t*>(buf_), sizeof(buf_));
        size_t n = rust_reader_read(reader_, slice);
        if (n == 0) return traits_type::eof();
        setg(buf_, buf_, buf_ + n);
        return traits_type::to_int_type(*gptr());
    }

    // Override to keep the vtable slot resolved within this TU instead of
    // referencing `std::basic_streambuf<char>::seekpos`, whose mangling depends
    // on `std::fpos<mbstate_t>` — and `mbstate_t` is a typedef to the internal
    // `_Mbstatet` on gcc 15 mingw but a different name on gcc 14, so the
    // external symbol fails to resolve when the prebuilt ships gcc 14
    // libstdc++.a but downstream links with gcc 15.
    pos_type seekpos(pos_type, std::ios_base::openmode = std::ios_base::in | std::ios_base::out) override {
        return pos_type(off_type(-1));
    }

private:
    RustReader& reader_;
    char buf_[8192];
};

// std::streambuf subclass that writes to a Rust `dyn Write` via FFI callback
class RustWriteStreambuf : public std::streambuf {
public:
    explicit RustWriteStreambuf(RustWriter& writer) : writer_(writer) {}

    ~RustWriteStreambuf() override {
        sync();
    }

protected:
    int_type overflow(int_type ch) override {
        if (ch != traits_type::eof()) {
            buf_[pos_++] = static_cast<char>(ch);
            if (pos_ >= sizeof(buf_)) {
                if (!flush_buf()) return traits_type::eof();
            }
        }
        return ch;
    }

    std::streamsize xsputn(const char* s, std::streamsize count) override {
        std::streamsize written = 0;
        while (written < count) {
            std::streamsize space = sizeof(buf_) - pos_;
            std::streamsize chunk = std::min(count - written, space);
            std::memcpy(buf_ + pos_, s + written, chunk);
            pos_ += static_cast<size_t>(chunk);
            written += chunk;
            if (pos_ >= sizeof(buf_)) {
                if (!flush_buf()) return written;
            }
        }
        return written;
    }

    int sync() override {
        return flush_buf() ? 0 : -1;
    }

    // See RustReadStreambuf::seekpos — same gcc 14/15 `_Mbstatet` mangling fix.
    pos_type seekpos(pos_type, std::ios_base::openmode = std::ios_base::in | std::ios_base::out) override {
        return pos_type(off_type(-1));
    }

private:
    bool flush_buf() {
        if (pos_ == 0) return true;
        rust::Slice<const uint8_t> slice(
            reinterpret_cast<const uint8_t*>(buf_), pos_);
        size_t n = rust_writer_write(writer_, slice);
        if (n < pos_) return false;
        pos_ = 0;
        return true;
    }

    RustWriter& writer_;
    char buf_[8192];
    size_t pos_ = 0;
};

std::unique_ptr<TopoDS_Shape> read_brep_stream(
    rust::Slice<const uint8_t> data, size_t& out_consumed)
{
    // istringstream because BinTools::Read seeks backwards to shared sub-shapes.
    std::istringstream iss(
        std::string(reinterpret_cast<const char*>(data.data()), data.size()));

    auto shape = std::make_unique<TopoDS_Shape>();
    try {
        BinTools::Read(*shape, iss);
    } catch (const Standard_Failure&) {
        return nullptr;  // out_consumed deliberately untouched
    }
    if (shape->IsNull()) {
        return nullptr;  // ditto
    }

    // clear() is load-bearing: a payload read to its last byte leaves eofbit set, and
    // tellg()'s sentry then turns that into failbit and returns -1.
    iss.clear();
    out_consumed = static_cast<size_t>(iss.tellg());
    return shape;
}

bool write_brep_stream(const TopoDS_Shape& shape, RustWriter& writer) {
    RustWriteStreambuf sbuf(writer);
    std::ostream os(&sbuf);
    try {
        // Pinned: the written bytes are a persisted checkpoint, so an OCCT bump
        // must not silently change the format `BinTools_FormatVersion_CURRENT` aliases.
        BinTools::Write(shape, os, false, false, BinTools_FormatVersion_VERSION_4);
    } catch (const Standard_Failure&) {
        return false;
    }
    return os.good();
}

#ifndef FEATURE_COLOR
// Plain STEP I/O — used only when FEATURE_COLOR is not defined.
std::unique_ptr<TopoDS_Shape> read_step_stream(RustReader& reader) {
    RustReadStreambuf sbuf(reader);
    std::istream is(&sbuf);

    STEPControl_Reader step_reader;
    IFSelect_ReturnStatus status = step_reader.ReadStream("stream", is);

    if (status != IFSelect_RetDone) {
        return nullptr;
    }

    step_reader.TransferRoots(Message_ProgressRange());
    return std::make_unique<TopoDS_Shape>(
        try_sew_orphan_faces(step_reader.OneShape(), nullptr));
}

bool write_step_stream(const TopoDS_Shape& shape, RustWriter& writer) {
    RustWriteStreambuf sbuf(writer);
    std::ostream os(&sbuf);
    STEPControl_Writer step_writer;
    if (step_writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone) {
        return false;
    }
    return step_writer.WriteStream(os) == IFSelect_RetDone;
}
#endif // !FEATURE_COLOR

} // namespace cadrum

#ifdef FEATURE_COLOR

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_ChildIterator.hxx>
#include <NCollection_Sequence.hxx>
#include <TDF_Label.hxx>
#include <Quantity_Color.hxx>

namespace cadrum {

// Face and solid keys share one map: a TShape* is unique across shape types. Solid
// color is NOT expanded onto faces — that would turn one STYLED_ITEM into N on write.
static void collect_colors(
    const Handle(TDocStd_Document)& doc,
    const Handle(XCAFDoc_ColorTool)& colorTool,
    std::unordered_map<uint64_t, std::array<float, 3>>& colorMap)
{
    for (TDF_ChildIterator it(doc->Main(), true); it.More(); it.Next()) {
        const TDF_Label& label = it.Value();
        if (!XCAFDoc_ShapeTool::IsShape(label)) continue;

        TopoDS_Shape s = XCAFDoc_ShapeTool::GetShape(label);
        if (s.IsNull()) continue;

        // Surface style first, generic style as the fallback.
        Quantity_Color color;
        if (colorTool->GetColor(label, XCAFDoc_ColorSurf, color) ||
            colorTool->GetColor(label, XCAFDoc_ColorGen, color)) {
            if (s.ShapeType() == TopAbs_FACE) {
                colorMap[reinterpret_cast<uint64_t>(s.TShape().get())] = {
                    (float)color.Red(), (float)color.Green(), (float)color.Blue()};
            } else {
                // A label's shape may be a COMPOUND/COMPSOLID — an assembly, or a
                // product of several bodies — which is a level STEP often styles.
                for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) {
                    colorMap[reinterpret_cast<uint64_t>(ex.Current().TShape().get())] = {
                        (float)color.Red(), (float)color.Green(), (float)color.Blue()};
                }
            }
        }
    }
}

std::unique_ptr<TopoDS_Shape> read_step_color_stream(
    RustReader&          reader,
    rust::Vec<uint64_t>& out_ids,
    rust::Vec<float>&    out_rgb)
{
    try {
        // Create XDE document directly — avoids XCAFApp_Application which
        // pulls in visualization libs (TKXCAFPrs/TKTPrsStd) built with
        // BUILD_MODULE_Visualization=OFF.  Handle<> ref-counts ownership.
        Handle(TDocStd_Document) doc = new TDocStd_Document("XmlXCAF");

        STEPCAFControl_Reader cafreader;
        cafreader.SetColorMode(true);

        RustReadStreambuf sbuf(reader);
        std::istream is(&sbuf);
        if (cafreader.ReadStream("stream", is) != IFSelect_RetDone) {
            return nullptr;
        }
        if (!cafreader.Transfer(doc)) {
            return nullptr;
        }

        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        Handle(XCAFDoc_ColorTool) colorTool =
            XCAFDoc_DocumentTool::ColorTool(doc->Main());

        // Collect all free shapes into a compound.
        NCollection_Sequence<TDF_Label> roots;
        shapeTool->GetFreeShapes(roots);

        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        for (int i = 1; i <= roots.Length(); i++) {
            builder.Add(compound, shapeTool->GetShape(roots.Value(i)));
        }

        std::unordered_map<uint64_t, std::array<float, 3>> colorMap;
        collect_colors(doc, colorTool, colorMap);

        // Recover Solids from disjoint shells / loose faces (#129); also remaps
        // colorMap keys for faces whose TShape* changed during sewing.
        TopoDS_Shape post = try_sew_orphan_faces(compound, &colorMap);

        // Walk the POST-processed shape so sewing's new TShape* are picked up, and
        // entries it no longer holds are dropped by not being reached.
        auto emit = [&](const TopoDS_Shape& sub) {
            uint64_t id = reinterpret_cast<uint64_t>(sub.TShape().get());
            auto it = colorMap.find(id);
            if (it == colorMap.end()) return;
            out_ids.push_back(id);
            out_rgb.push_back(it->second[0]);
            out_rgb.push_back(it->second[1]);
            out_rgb.push_back(it->second[2]);
        };
        for (TopExp_Explorer ex(post, TopAbs_FACE); ex.More(); ex.Next()) {
            emit(ex.Current());
        }
        for (TopExp_Explorer ex(post, TopAbs_SOLID); ex.More(); ex.Next()) {
            emit(ex.Current());
        }

        return std::make_unique<TopoDS_Shape>(post);
    } catch (const Standard_Failure&) {
        return nullptr;
    }
}

bool write_step_color_stream(
    const TopoDS_Shape&         shape,
    rust::Slice<const uint64_t> ids,
    rust::Slice<const float>    rgb,
    RustWriter&                 writer)
{
    try {
        Handle(TDocStd_Document) doc = new TDocStd_Document("XmlXCAF");

        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        Handle(XCAFDoc_ColorTool) colorTool =
            XCAFDoc_DocumentTool::ColorTool(doc->Main());

        // Register the root shape.
        TDF_Label rootLabel = shapeTool->AddShape(shape, false);

        // One lookup for both levels: which explorer finds an id decides the level
        // it is written at.
        std::unordered_map<uint64_t, std::array<float, 3>> colorLookup;
        for (size_t i = 0; i < ids.size(); i++) {
            colorLookup[ids[i]] = {rgb[3*i], rgb[3*i+1], rgb[3*i+2]};
        }

        // Find/create the sub-shape label of `sub` and paint it.
        auto set_color = [&](const TopoDS_Shape& sub, const std::array<float, 3>& c) {
            TDF_Label label;
            if (!shapeTool->FindSubShape(rootLabel, sub, label)) {
                label = shapeTool->AddSubShape(rootLabel, sub);
            }
            Quantity_Color color(c[0], c[1], c[2], Quantity_TOC_RGB);
            colorTool->SetColor(label, color, XCAFDoc_ColorSurf);
        };

        // Solids first: a face style is the more specific one and must be set after.
        for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) {
            const TopoDS_Shape& solid = ex.Current();
            auto it = colorLookup.find(
                reinterpret_cast<uint64_t>(solid.TShape().get()));
            if (it == colorLookup.end()) continue;
            set_color(solid, it->second);
        }

        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
            const TopoDS_Shape& face = ex.Current();
            auto it = colorLookup.find(
                reinterpret_cast<uint64_t>(face.TShape().get()));
            if (it == colorLookup.end()) continue;
            set_color(face, it->second);
        }

        // Transfer XDE doc to STEP model and write to stream.
        STEPCAFControl_Writer cafwriter;
        cafwriter.SetColorMode(true);
        if (!cafwriter.Transfer(doc)) {
            return false;
        }

        RustWriteStreambuf sbuf(writer);
        std::ostream os(&sbuf);
        return cafwriter.ChangeWriter().WriteStream(os) == IFSelect_RetDone;
    } catch (const Standard_Failure&) {
        return false;
    }
}

} // namespace cadrum

#endif // FEATURE_COLOR
