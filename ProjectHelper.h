#pragma once

#include <vector>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>

/// 构造"世界坐标系 -> 轴测视图坐标系"的刚体变换（HLR 投影器与脊线等任意
/// 3D 元素的投影必须共用本变换，保证投影结果同源、严格重合）。
/// 视图系：Z = 视线方向 projDir（深度），X = projUp × projDir（屏幕右），
/// Y = Z × X（屏幕上，等于 projUp 在视平面上的归一化投影）。
/// 对任意点 p 做 p.Transform(outTrsf) 后，其 X/Y 即与 HLR 输出一致的 2D 坐标。
/// projUp 与 projDir 平行（无法构成视图系）时返回 false。
bool BuildViewTransform(const gp_Dir& projDir,
    const gp_Dir& projUp,
    gp_Trsf& outTrsf);

/// 单条投影多段线结果
struct ProjectedPolyline2D
{
    std::vector<gp_Pnt2d> points;
};

/// 投影曲线类型（解析形式，非离散化）
enum class ProjectedCurve2DType
{
    Line,    // 线段
    Arc,     // 圆弧
    Circle   // 整圆
};

/// 单条投影曲线结果（解析形式，保留线段/圆弧/圆的几何参数）
struct ProjectedCurve2D
{
    ProjectedCurve2DType type;

    // 线段：startPoint / endPoint 为两端点
    // 圆弧：startPoint / endPoint 为弧两端点；center 为圆心；radius 为半径；
    //       startAngle / endAngle 为起止角（弧度）；isClockwise 为绘制方向
    // 圆：  center 为圆心；radius 为半径
    gp_Pnt2d startPoint;
    gp_Pnt2d endPoint;
    gp_Pnt2d center;
    double radius;
    double startAngle;   // 弧度
    double endAngle;     // 弧度
    bool isClockwise;    // 圆弧方向（true=顺时针，false=逆时针）
};

bool ProjectBoxSetTo2DCurves(const std::vector<TopoDS_Shape>& boxShapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedPolyline2D>& outCurves);

/// 将 Shape 集合沿指定方向投影，输出解析形式的 2D 曲线（线段/圆弧/圆），不离散化
bool ProjectCurveSetTo2DCurves(const std::vector<TopoDS_Shape>& shapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedCurve2D>& outCurves);

/// 基于三角化多面体的 HLR 投影（HLRBRep_PolyAlgo）：
/// 先对输入 shape 做三角剖分（BRepMesh），再以多面体边/三角形计算消隐，
/// 输出全部为 2D 折线（每条可见边一条折线；曲面轮廓由折线段逼近）。
/// 与 ProjectCurveSetTo2DCurves 共用同一权威视图变换（BuildViewTransform），
/// 投影坐标同源。三角化是附加显示数据，不改变解析几何，不影响原 HLR 流程。
bool ProjectShapeTo2DByPolyHLR(const std::vector<TopoDS_Shape>& shapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedPolyline2D>& outPolylines);


bool IsShapeValid(const TopoDS_Shape& shape);
TopoDS_Shape FixShape(const TopoDS_Shape& shape);

TopoDS_Face BuildFaceWithHoles(const TopoDS_Wire& outerWire,
    const std::vector<TopoDS_Wire>& holeWires,
    const gp_Pnt& planeOrigin,
    const gp_Dir& planeNormal,
    Standard_Real prec = 1e-6);

/// 生成扫掠线框骨架：两条脊线分别与闭合截面轮廓做 PipeShell 扫掠，
/// 提取扫掠壳中所有 Edge 组装成 Compound，并补全 3D 曲线，
/// 结果可直接用于 HLR 三维遮挡/投影计算。
TopoDS_Shape BuildSweepWireframe(const TopoDS_Wire& spineWire1,
    const TopoDS_Wire& spineWire2,
    const TopoDS_Wire& sectionWire);

/// 生成带面封闭扫掠壳：闭合截面轮廓沿脊线扫掠（PipeShell）生成侧面壳，
/// 提取两端自由边界补平面端盖并缝合，返回两端封闭、含全部面的壳。
/// 面参与 HLR 遮挡判定——纯边线框无面、无遮挡效果，需要消隐的轴测投影
/// 必须传入本函数的结果。
TopoDS_Shape BuildSweepShell(const TopoDS_Wire& spineWire,
    const TopoDS_Wire& sectionWire);
