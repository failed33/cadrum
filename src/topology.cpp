// The topology report and the parametric queries beside it. Every column is
// read once per shape in `TopExp::MapShapes` order, which is the order the
// display mesh and the lineage speak as well. The report's rows are the
// shared structs of `ffi.rs`, filled by field name.

#include "cadrum/src/ffi.rs.h"
#include "cadrum/src/ffi.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Surface.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Elips.hxx>
#include <gp_Hypr.hxx>
#include <gp_Lin.hxx>
#include <gp_Parab.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

#include <cmath>
#include <limits>
#include <utility>

namespace cadrum {

using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using AncestorMap = NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

uint64_t shape_tshape(const TopoDS_Shape& shape) {
    return reinterpret_cast<uint64_t>(shape.TShape().get());
}

uint64_t shape_location(const TopoDS_Shape& shape) {
    return static_cast<uint64_t>(std::hash<TopLoc_Location>{}(shape.Location()));
}

void shape_key(const TopoDS_Shape& shape, uint64_t& tshape, uint64_t& location) {
    tshape = shape_tshape(shape);
    location = shape_location(shape);
}

void face_key(const TopoDS_Face& face, uint64_t& tshape, uint64_t& location) {
    shape_key(face, tshape, location);
}

void edge_key(const TopoDS_Edge& edge, uint64_t& tshape, uint64_t& location) {
    shape_key(edge, tshape, location);
}

namespace {

constexpr uint32_t NO_INDEX = std::numeric_limits<uint32_t>::max();
constexpr double NAN_COORD = std::numeric_limits<double>::quiet_NaN();

Xyz xyz(const gp_XYZ& value) {
    return Xyz{.x = value.X(), .y = value.Y(), .z = value.Z()};
}

Xyz unit_or_zero(const gp_Vec& vector) {
    return vector.Magnitude() > Precision::Confusion() ? xyz(vector.Normalized().XYZ()) : Xyz{.x = 0.0, .y = 0.0, .z = 0.0};
}

// `gp_Ax2` and `gp_Ax3` alike: location, main direction and x direction.
template <typename Axes>
PlacementData placement(const Axes& axes) {
    return PlacementData{.origin = xyz(axes.Location().XYZ()), .axis = xyz(axes.Direction().XYZ()), .reference = xyz(axes.XDirection().XYZ())};
}

AxisData axis(const gp_Ax1& value) {
    return AxisData{.origin = xyz(value.Location().XYZ()), .direction = xyz(value.Direction().XYZ())};
}

KeyData key(const TopoDS_Shape& shape) {
    return KeyData{.tshape = shape_tshape(shape), .location = shape_location(shape)};
}

// The row `value` now occupies in `column`.
template <typename Row>
uint32_t append(rust::Vec<Row>& column, Row value) {
    column.push_back(value);
    return static_cast<uint32_t>(column.size() - 1);
}

uint32_t index_of(const ShapeMap& map, const TopoDS_Shape& shape) {
    const int index = map.FindIndex(shape);
    return index > 0 ? static_cast<uint32_t>(index - 1) : NO_INDEX;
}

// The definition of `surface`, pushed into the column of its kind: the kind
// and the row it occupies there.
std::pair<SurfaceCode, uint32_t> surface_definition(TopologyData& out, const BRepAdaptor_Surface& surface) {
    switch (surface.GetType()) {
        case GeomAbs_Plane:
            return {SurfaceCode::Plane, append(out.planes, PlaneDef{.placement = placement(surface.Plane().Position())})};
        case GeomAbs_Cylinder: {
            const gp_Cylinder cylinder = surface.Cylinder();
            return {SurfaceCode::Cylinder, append(out.cylinders, CylinderDef{.placement = placement(cylinder.Position()), .radius = cylinder.Radius()})};
        }
        case GeomAbs_Cone: {
            const gp_Cone cone = surface.Cone();
            return {SurfaceCode::Cone, append(out.cones, ConeDef{.placement = placement(cone.Position()), .radius = cone.RefRadius(), .semi_angle = cone.SemiAngle()})};
        }
        case GeomAbs_Sphere: {
            const gp_Sphere sphere = surface.Sphere();
            return {SurfaceCode::Sphere, append(out.spheres, SphereDef{.placement = placement(sphere.Position()), .radius = sphere.Radius()})};
        }
        case GeomAbs_Torus: {
            const gp_Torus torus = surface.Torus();
            return {SurfaceCode::Torus, append(out.tori, TorusDef{.placement = placement(torus.Position()), .major_radius = torus.MajorRadius(), .minor_radius = torus.MinorRadius()})};
        }
        case GeomAbs_BezierSurface: {
            const Handle(Geom_BezierSurface) bezier = surface.Bezier();
            return {SurfaceCode::Bezier, append(out.spline_surfaces, SplineSurfaceDef{
                .u_degree = static_cast<uint32_t>(bezier->UDegree()), .v_degree = static_cast<uint32_t>(bezier->VDegree()),
                .u_poles = static_cast<uint32_t>(bezier->NbUPoles()), .v_poles = static_cast<uint32_t>(bezier->NbVPoles()),
                .u_periodic = false, .v_periodic = false, .rational = bezier->IsURational() || bezier->IsVRational()})};
        }
        case GeomAbs_BSplineSurface: {
            const Handle(Geom_BSplineSurface) spline = surface.BSpline();
            return {SurfaceCode::BSpline, append(out.spline_surfaces, SplineSurfaceDef{
                .u_degree = static_cast<uint32_t>(spline->UDegree()), .v_degree = static_cast<uint32_t>(spline->VDegree()),
                .u_poles = static_cast<uint32_t>(spline->NbUPoles()), .v_poles = static_cast<uint32_t>(spline->NbVPoles()),
                .u_periodic = spline->IsUPeriodic(), .v_periodic = spline->IsVPeriodic(), .rational = spline->IsURational() || spline->IsVRational()})};
        }
        case GeomAbs_SurfaceOfRevolution:
            return {SurfaceCode::Revolution, append(out.revolutions, RevolutionDef{.axis = axis(surface.AxeOfRevolution())})};
        case GeomAbs_SurfaceOfExtrusion:
            return {SurfaceCode::Extrusion, append(out.extrusions, ExtrusionDef{.direction = xyz(surface.Direction().XYZ())})};
        case GeomAbs_OffsetSurface:
            return {SurfaceCode::Offset, append(out.surface_offsets, OffsetDef{.offset = surface.OffsetValue()})};
        case GeomAbs_OtherSurface:
            break;
    }
    return {SurfaceCode::Other, 0};
}

std::pair<CurveCode, uint32_t> curve_definition(TopologyData& out, const BRepAdaptor_Curve& curve) {
    switch (curve.GetType()) {
        case GeomAbs_Line:
            return {CurveCode::Line, append(out.lines, LineDef{.axis = axis(curve.Line().Position())})};
        case GeomAbs_Circle: {
            const gp_Circ circle = curve.Circle();
            return {CurveCode::Circle, append(out.circles, CircleDef{.placement = placement(circle.Position()), .radius = circle.Radius()})};
        }
        case GeomAbs_Ellipse: {
            const gp_Elips ellipse = curve.Ellipse();
            return {CurveCode::Ellipse, append(out.ellipses, EllipseDef{.placement = placement(ellipse.Position()), .major_radius = ellipse.MajorRadius(), .minor_radius = ellipse.MinorRadius()})};
        }
        case GeomAbs_Hyperbola: {
            const gp_Hypr hyperbola = curve.Hyperbola();
            return {CurveCode::Hyperbola, append(out.hyperbolas, HyperbolaDef{.placement = placement(hyperbola.Position()), .major_radius = hyperbola.MajorRadius(), .minor_radius = hyperbola.MinorRadius()})};
        }
        case GeomAbs_Parabola: {
            const gp_Parab parabola = curve.Parabola();
            return {CurveCode::Parabola, append(out.parabolas, ParabolaDef{.placement = placement(parabola.Position()), .focal = parabola.Focal()})};
        }
        case GeomAbs_BezierCurve: {
            const Handle(Geom_BezierCurve) bezier = curve.Bezier();
            return {CurveCode::Bezier, append(out.spline_curves, SplineCurveDef{.degree = static_cast<uint32_t>(bezier->Degree()), .poles = static_cast<uint32_t>(bezier->NbPoles()), .periodic = false, .rational = bezier->IsRational()})};
        }
        case GeomAbs_BSplineCurve: {
            const Handle(Geom_BSplineCurve) spline = curve.BSpline();
            return {CurveCode::BSpline, append(out.spline_curves, SplineCurveDef{.degree = static_cast<uint32_t>(spline->Degree()), .poles = static_cast<uint32_t>(spline->NbPoles()), .periodic = spline->IsPeriodic(), .rational = spline->IsRational()})};
        }
        case GeomAbs_OffsetCurve:
            return {CurveCode::Offset, append(out.curve_offsets, OffsetDef{.offset = curve.OffsetCurve()->Offset()})};
        case GeomAbs_OtherCurve:
            break;
    }
    return {CurveCode::Other, 0};
}

void face_columns(TopologyData& out, const TopoDS_Face& face, const ShapeMap& edges) {
    out.face_keys.push_back(key(face));
    const BRepAdaptor_Surface surface(face, false);
    const auto [kind, row] = surface_definition(out, surface);
    out.face_surface.push_back(kind);
    out.face_definition.push_back(row);
    out.face_reversed.push_back(face.Orientation() == TopAbs_REVERSED);
    out.face_tolerance.push_back(BRep_Tool::Tolerance(face));
    ShapeMap bounding;
    TopExp::MapShapes(face, TopAbs_EDGE, bounding);
    for (int index = 1; index <= bounding.Extent(); ++index) out.face_edges.push_back(index_of(edges, bounding(index)));
    out.face_edge_offsets.push_back(static_cast<uint32_t>(out.face_edges.size()));
}

void edge_columns(TopologyData& out, const TopoDS_Edge& edge, const ShapeMap& vertices, const ShapeMap& faces, const AncestorMap& ancestors) {
    out.edge_keys.push_back(key(edge));
    double first = 0.0, last = 0.0;
    const bool curved = !BRep_Tool::Degenerated(edge) && !BRep_Tool::Curve(edge, first, last).IsNull();
    if (curved) {
        const BRepAdaptor_Curve curve(edge);
        const auto [kind, row] = curve_definition(out, curve);
        out.edge_curve.push_back(kind);
        out.edge_definition.push_back(row);
        gp_Pnt start, end;
        gp_Vec at_start, at_end;
        curve.D1(curve.FirstParameter(), start, at_start);
        curve.D1(curve.LastParameter(), end, at_end);
        out.edge_ends.push_back(EdgeEnds{.start = xyz(start.XYZ()), .start_tangent = unit_or_zero(at_start), .end = xyz(end.XYZ()), .end_tangent = unit_or_zero(at_end)});
        out.edge_length.push_back(GCPnts_AbscissaPoint::Length(curve));
    } else {
        out.edge_curve.push_back(CurveCode::Degenerate);
        out.edge_definition.push_back(0);
        out.edge_ends.push_back(EdgeEnds{});
        out.edge_length.push_back(0.0);
    }
    out.edge_tolerance.push_back(BRep_Tool::Tolerance(edge));
    TopoDS_Vertex start, end;
    TopExp::Vertices(edge, start, end, false);
    out.edge_vertices.push_back(start.IsNull() ? NO_INDEX : index_of(vertices, start));
    out.edge_vertices.push_back(end.IsNull() ? NO_INDEX : index_of(vertices, end));
    if (const NCollection_List<TopoDS_Shape>* adjacent = ancestors.Seek(edge)) {
        ShapeMap unique;
        for (NCollection_List<TopoDS_Shape>::Iterator it(*adjacent); it.More(); it.Next()) unique.Add(it.Value());
        for (int index = 1; index <= unique.Extent(); ++index) out.edge_faces.push_back(index_of(faces, unique(index)));
    }
    out.edge_face_offsets.push_back(static_cast<uint32_t>(out.edge_faces.size()));
}

}  // namespace

TopologyData shape_topology(const TopoDS_Shape& shape) {
    TopologyData out{};
    ShapeMap faces, edges, vertices;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    AncestorMap ancestors;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ancestors);

    out.face_edge_offsets.push_back(0);
    for (int index = 1; index <= faces.Extent(); ++index) face_columns(out, TopoDS::Face(faces(index)), edges);
    out.edge_face_offsets.push_back(0);
    for (int index = 1; index <= edges.Extent(); ++index) edge_columns(out, TopoDS::Edge(edges(index)), vertices, faces, ancestors);
    for (int index = 1; index <= vertices.Extent(); ++index) {
        const TopoDS_Vertex vertex = TopoDS::Vertex(vertices(index));
        out.vertex_keys.push_back(key(vertex));
        out.vertex_points.push_back(xyz(BRep_Tool::Pnt(vertex).XYZ()));
        out.vertex_tolerance.push_back(BRep_Tool::Tolerance(vertex));
    }

    Bnd_Box box;
    BRepBndLib::AddOptimal(shape, box, false, false);
    if (box.IsVoid()) BRepBndLib::Add(shape, box, false);
    out.bounded = !box.IsVoid();
    if (out.bounded) {
        double low_x, low_y, low_z, high_x, high_y, high_z;
        box.Get(low_x, low_y, low_z, high_x, high_y, high_z);
        out.extent = ExtentData{.low = Xyz{.x = low_x, .y = low_y, .z = low_z}, .high = Xyz{.x = high_x, .y = high_y, .z = high_z}};
    }
    return out;
}

NearestData shape_nearest(const TopoDS_Shape& shape, double x, double y, double z) {
    NearestData out{0, 0, 0, NAN_COORD, NAN_COORD, NAN_COORD, NAN_COORD, NAN_COORD, NAN_COORD};
    const TopoDS_Vertex probe = BRepBuilderAPI_MakeVertex(gp_Pnt(x, y, z));
    BRepExtrema_DistShapeShape extrema(probe, shape, Extrema_ExtFlag_MIN);
    if (!extrema.IsDone() || extrema.NbSolution() < 1) return out;
    // Every solution is at the minimum distance; a face support says the most.
    int chosen = 1;
    for (int solution = 1; solution <= extrema.NbSolution(); ++solution) {
        if (extrema.SupportTypeShape2(solution) == BRepExtrema_IsInFace) {
            chosen = solution;
            break;
        }
    }
    const TopoDS_Shape support = extrema.SupportOnShape2(chosen);
    switch (extrema.SupportTypeShape2(chosen)) {
        case BRepExtrema_IsVertex: out.support = 1; break;
        case BRepExtrema_IsOnEdge: out.support = 2; break;
        case BRepExtrema_IsInFace: out.support = 3; break;
    }
    out.tshape = shape_tshape(support);
    out.location = shape_location(support);
    const gp_Pnt point = extrema.PointOnShape2(chosen);
    out.px = point.X(); out.py = point.Y(); out.pz = point.Z();
    // The normal of the face the hit lies on: the support itself, or the
    // first face bounded by the supporting edge or vertex.
    TopoDS_Face face;
    double u = 0.0, v = 0.0;
    if (out.support == 3) {
        face = TopoDS::Face(support);
        extrema.ParOnFaceS2(chosen, u, v);
    } else {
        AncestorMap ancestors;
        TopExp::MapShapesAndAncestors(shape, support.ShapeType(), TopAbs_FACE, ancestors);
        const NCollection_List<TopoDS_Shape>* adjacent = ancestors.Seek(support);
        if (!adjacent || adjacent->IsEmpty()) return out;
        face = TopoDS::Face(adjacent->First());
        TopLoc_Location location;
        const Handle(Geom_Surface) geometry = BRep_Tool::Surface(face, location);
        if (geometry.IsNull()) return out;
        GeomAPI_ProjectPointOnSurf projector(point.Transformed(location.Transformation().Inverted()), geometry);
        if (!projector.IsDone() || projector.NbPoints() < 1) return out;
        projector.LowerDistanceParameters(u, v);
    }
    BRepAdaptor_Surface surface(face);
    BRepLProp_SLProps properties(surface, u, v, 1, Precision::Confusion());
    if (properties.IsNormalDefined()) {
        gp_Dir normal = properties.Normal();
        if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
        out.nx = normal.X(); out.ny = normal.Y(); out.nz = normal.Z();
    }
    return out;
}

bool edge_at_length(const TopoDS_Edge& edge, double distance, rust::Slice<double> out) {
    if (out.size() != 7 || !std::isfinite(distance)) return false;
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    GCPnts_AbscissaPoint station(curve, distance, first);
    if (!station.IsDone()) return false;
    const double parameter = std::min(std::max(station.Parameter(), first), last);
    gp_Pnt point;
    gp_Vec derivative;
    curve.D1(parameter, point, derivative);
    const Xyz tangent = unit_or_zero(derivative);
    out[0] = point.X(); out[1] = point.Y(); out[2] = point.Z();
    out[3] = tangent.x; out[4] = tangent.y; out[5] = tangent.z;
    out[6] = last > first ? (parameter - first) / (last - first) : 0.0;
    return true;
}

}  // namespace cadrum
