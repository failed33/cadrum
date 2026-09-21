// Reproduce OCCT 8.0.1's spline-extrusion integral error without application code.
#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <NCollection_HArray1.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Pln.hxx>
#include <cmath>
#include <iostream>

int main() {
    Handle(NCollection_HArray1<gp_Pnt>) points = new NCollection_HArray1<gp_Pnt>(1, 16);
    for (int i = 0; i < 16; ++i) {
        const double angle = 2.0 * std::acos(-1.0) * i / 16;
        points->SetValue(i + 1, gp_Pnt(2 * std::cos(angle), std::sin(angle), 0));
    }
    GeomAPI_Interpolate interpolate(points, true, 1e-6);
    interpolate.Perform();
    const auto curve = interpolate.Curve();
    const auto wire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(curve).Edge()).Wire();
    const auto face = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0), gp_Dir(0,0,1)), wire, true).Face();
    const auto body = BRepPrimAPI_MakePrism(face, gp_Vec(0,0,3)).Shape();
    GProp_GProps properties;
    const double error = BRepGProp::VolumeProperties(body, properties, 1e-8);
    double twice_area = 0;
    auto previous = curve->Value(curve->FirstParameter());
    for (int i = 1; i <= 65536; ++i) {
        const auto current = curve->Value(curve->FirstParameter() +
            (curve->LastParameter() - curve->FirstParameter()) * i / 65536.0);
        twice_area += previous.X() * current.Y() - previous.Y() * current.X();
        previous = current;
    }
    const double reference = std::abs(twice_area) * 1.5;
    std::cout.precision(17);
    std::cout << "native=" << properties.Mass() << " reference=" << reference
              << " reported_error=" << error << '\n';
    return std::abs(properties.Mass() / reference - 1.0) < 1e-4 ? 0 : 1;
}
