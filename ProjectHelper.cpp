#include "ProjectHelper.h"

#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <HLRBRep_PolyAlgo.hxx>
#include <HLRBRep_PolyHLRToShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_CurveType.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>

#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Wire.hxx>
#include <Geom_Plane.hxx>
#include <BRep_Tool.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepBuilderAPI.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <NCollection_Map.hxx>
#include <TopoDS_Compound.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <iostream>

// 权威视图变换：HLR 投影器与脊线等任意 3D 元素的投影共用，见头文件说明
bool BuildViewTransform(const gp_Dir& projDir,
    const gp_Dir& projUp,
    gp_Trsf& outTrsf)
{
    // 屏幕右方向 = up × dir（gp_Ax3 第三参数是 X 方向，不能直接传 projUp）
    gp_Vec rightVec = gp_Vec(projUp).Crossed(gp_Vec(projDir));
    if (rightVec.Magnitude() < 1.0e-9)
    {
        std::cout << "错误：上方向 projUp 与视线方向 projDir 平行，无法构成视图坐标系" << std::endl;
        return false;
    }
    gp_Dir rightDir(rightVec);

    // 世界系 -> 视图系：变换后点的 z=沿视线的深度，X/Y=屏幕坐标，丢弃 z 即正交投影。
    // 方向不能反：viewSys->世界系会使丢弃的 z 变成 world-Z 方向，投影方向错误。
    gp_Ax3 viewSys(gp::Origin(), projDir, rightDir);
    outTrsf.SetTransformation(gp_Ax3(), viewSys);
    return true;
}

bool ProjectBoxSetTo2DCurves(const std::vector<TopoDS_Shape>& boxShapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedPolyline2D>& outCurves)
{
    outCurves.clear();
    if (boxShapes.empty())
    {
        std::cout << "调试：输入shape集合为空" << std::endl;
        return false;
    }

    // 权威视图变换（世界系 -> 视图系），与脊线投影等共用同一变换
    gp_Trsf trsf;
    if (!BuildViewTransform(projDir, projUp, trsf))
        return false;
    HLRAlgo_Projector projector(trsf, Standard_False, 0.0);

    Handle(HLRBRep_Algo) hlrAlgo = new HLRBRep_Algo();
    hlrAlgo->Projector(projector);

    int addCount = 0;
    for (const auto& shape : boxShapes)
    {
        if (!shape.IsNull())
        {
            hlrAlgo->Add(shape);
            addCount++;
        }
    }
    std::cout << "调试：Add()添加实体数量=" << addCount << std::endl;

    hlrAlgo->Update();
    std::cout << "调试：Update()执行完成" << std::endl;

    // 关键：Update() 只把几何体投影进内部数据结构并计算轮廓，
    // Hide() 才真正做面遮挡判定，将边分类为可见(VCompound)/隐藏(HCompound)。
    // 不调用 Hide() 时所有边都会按可见返回，遮挡消隐不生效。
    hlrAlgo->Hide();
    std::cout << "调试：Hide()遮挡判定完成" << std::endl;

    HLRBRep_HLRToShape hlrConverter(hlrAlgo);
    TopoDS_Shape visibleEdgesCompound = hlrConverter.VCompound();

    // G1 相切边单独存放于 Rg1LineVCompound（VCompound 只含锐边），合并后统一遍历，
    // 使扫掠体中截面轮廓相切连接点位置的纵向棱线也能被离散输出
    TopoDS_Shape rg1Edges = hlrConverter.Rg1LineVCompound();

    // 轮廓边单独存放于 OutLineVCompound：圆弧面与投影方向相切的母线
    // （曲面沿视线方向的轮廓投影），如圆柱面切线棱线。合并后可输出完整轮廓
    TopoDS_Shape outlineEdges = hlrConverter.OutLineVCompound();

    // 遮挡（消隐）能力验证：HLRBRep_Algo::Update() 已按投影器完成可见性分类，
    // VCompound() 返回未被遮挡的边，HCompound() 返回被遮挡的边（隐藏线）。
    TopoDS_Shape hiddenEdgesCompound = hlrConverter.HCompound();
    int hiddenEdgeCount = 0;
    if (!hiddenEdgesCompound.IsNull())
    {
        for (TopExp_Explorer hExp(hiddenEdgesCompound, TopAbs_EDGE); hExp.More(); hExp.Next())
            hiddenEdgeCount++;
    }
    std::cout << "调试：遮挡消隐 -> 被遮挡的隐藏边(HCompound)数量=" << hiddenEdgeCount
              << "（Hide()已执行，VCompound 仅保留未被遮挡的可见边）" << std::endl;

    if (visibleEdgesCompound.IsNull() && rg1Edges.IsNull() && outlineEdges.IsNull())
    {
        std::cout << "调试：VCompound/Rg1Line/OutLine 均返回NULL！没有可见边" << std::endl;
        TopoDS_Shape hidden = hlrConverter.HCompound();
        if (hidden.IsNull())
            std::cout << "调试：HCompound()也是NULL，HLR计算完全没有输出" << std::endl;
        else
            std::cout << "调试：HCompound()不为空，只有遮挡边，无可见边" << std::endl;
        return false;
    }

    // 合并锐边、G1 相切边、轮廓边后统一遍历，NCollection_Map 去重
    TopoDS_Compound allVisible;
    BRep_Builder builder;
    builder.MakeCompound(allVisible);
    NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> seen;
    int rg1EdgeCount = 0, outlineEdgeCount = 0;
    auto addEdges = [&](const TopoDS_Shape& comp, int* counter) {
        if (comp.IsNull())
            return;
        for (TopExp_Explorer eExp(comp, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            if (seen.Add(eExp.Current()))
            {
                builder.Add(allVisible, eExp.Current());
                if (counter) (*counter)++;
            }
        }
    };
    addEdges(visibleEdgesCompound, nullptr);
    addEdges(rg1Edges, &rg1EdgeCount);
    addEdges(outlineEdges, &outlineEdgeCount);
    std::cout << "调试：VCompound有效（含G1相切边" << rg1EdgeCount
              << "条、轮廓边" << outlineEdgeCount << "条），开始遍历边" << std::endl;

    // 边遍历：按曲线类型分别处理。
    // 几何说明：HLRBRep_HLRToShape 输出的几何已经位于投影器坐标系中（视图空间，
    // x/y 即 2D 投影坐标，z 为深度），故对原始曲线采样后直接取 X/Y 即可；
    // 不能再调用 projector.Project()，否则会对视图空间坐标二次变换（结果错误）。
    TopExp_Explorer exp;
    int edgeTotal = 0;
    int lineCount = 0;     // 直线边数
    int curveCount = 0;    // 圆弧/圆/椭圆/BSpline 等曲线边数
    int skippedCount = 0;  // 退化/参数非法等跳过边数
    for (exp.Init(allVisible, TopAbs_EDGE); exp.More(); exp.Next())
    {
        edgeTotal++;
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());

        // 跳过退化边（曲面参数退化处，无几何意义）
        if (BRep_Tool::Degenerated(edge))
        {
            skippedCount++;
            continue;
        }

        BRepAdaptor_Curve curveAdaptor(edge);
        GeomAbs_CurveType curveType = curveAdaptor.GetType();

        double firstParam = curveAdaptor.FirstParameter();
        double lastParam = curveAdaptor.LastParameter();
        if (lastParam <= firstParam)
        {
            skippedCount++;
            continue;
        }

        ProjectedPolyline2D polyLine;

        if (curveType == GeomAbs_Line)
        {
            // 直线：取两端顶点，2 个点即可
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2);
            if (v1.IsNull() || v2.IsNull())
            {
                skippedCount++;
                continue;
            }
            gp_Pnt p3d_1 = BRep_Tool::Pnt(v1);
            gp_Pnt p3d_2 = BRep_Tool::Pnt(v2);
            polyLine.points.push_back(gp_Pnt2d(p3d_1.X(), p3d_1.Y()));
            polyLine.points.push_back(gp_Pnt2d(p3d_2.X(), p3d_2.Y()));
            lineCount++;
        }
        else
        {
            // 圆弧/圆/椭圆/BSpline 等：按参数范围离散采样
            // 采样密度：约 5 度（0.0873 rad）/点，最少 4 点，最多 360 点
            double paramRange = lastParam - firstParam;
            int nbPoints = (int)(paramRange / 0.0873) + 1;
            if (nbPoints < 4) nbPoints = 4;
            if (nbPoints > 360) nbPoints = 360;

            for (int i = 0; i < nbPoints; ++i)
            {
                double param = firstParam + paramRange * i / (nbPoints - 1);
                gp_Pnt p3d = curveAdaptor.Value(param);
                polyLine.points.push_back(gp_Pnt2d(p3d.X(), p3d.Y()));
            }
            curveCount++;
        }

        outCurves.push_back(polyLine);
    }
    std::cout << "调试：VCompound总Edge数量=" << edgeTotal
              << "（直线=" << lineCount << " 曲线=" << curveCount
              << " 跳过=" << skippedCount << "）" << std::endl;
    std::cout << "调试：遍历结束，输出曲线数量=" << outCurves.size() << std::endl;

    return !outCurves.empty();
}

bool ProjectCurveSetTo2DCurves(const std::vector<TopoDS_Shape>& shapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedCurve2D>& outCurves)
{
    outCurves.clear();
    if (shapes.empty())
    {
        std::cout << "调试：输入shape集合为空" << std::endl;
        return false;
    }

    // 权威视图变换（世界系 -> 视图系），与脊线投影等共用同一变换
    gp_Trsf trsf;
    if (!BuildViewTransform(projDir, projUp, trsf))
        return false;
    HLRAlgo_Projector projector(trsf, Standard_False, 0.0);

    Handle(HLRBRep_Algo) hlrAlgo = new HLRBRep_Algo();
    hlrAlgo->Projector(projector);

    int addCount = 0;
    for (const auto& shape : shapes)
    {
        if (!shape.IsNull())
        {
            hlrAlgo->Add(shape);
            addCount++;
        }
    }
    std::cout << "调试：Add()添加实体数量=" << addCount << std::endl;

    hlrAlgo->Update();
    std::cout << "调试：Update()执行完成" << std::endl;

    hlrAlgo->Hide();
    std::cout << "调试：Hide()遮挡判定完成" << std::endl;

    HLRBRep_HLRToShape hlrConverter(hlrAlgo);
    TopoDS_Shape visibleEdgesCompound = hlrConverter.VCompound();

    // G1 相切边单独存放于 Rg1LineVCompound（VCompound 只含锐边）。
    // 扫掠体中，截面轮廓圆弧与线段相切连接点位置的纵向棱线是面间平滑共享边，
    // 必须合并 Rg1LineVCompound 才能在投影结果中输出这些棱线。
    TopoDS_Shape rg1Edges = hlrConverter.Rg1LineVCompound();

    // 轮廓边单独存放于 OutLineVCompound：圆弧面与投影方向相切的母线
    // （曲面沿视线方向的轮廓投影），如圆柱面切线棱线
    TopoDS_Shape outlineEdges = hlrConverter.OutLineVCompound();

    TopoDS_Shape hiddenEdgesCompound = hlrConverter.HCompound();
    int hiddenEdgeCount = 0;
    if (!hiddenEdgesCompound.IsNull())
    {
        for (TopExp_Explorer hExp(hiddenEdgesCompound, TopAbs_EDGE); hExp.More(); hExp.Next())
            hiddenEdgeCount++;
    }
    std::cout << "调试：遮挡消隐 -> 被遮挡的隐藏边(HCompound)数量=" << hiddenEdgeCount << std::endl;

    if (visibleEdgesCompound.IsNull() && rg1Edges.IsNull() && outlineEdges.IsNull())
    {
        std::cout << "调试：VCompound/Rg1Line/OutLine 均返回NULL！没有可见边" << std::endl;
        return false;
    }

    // 合并锐边、G1 相切边、轮廓边后统一遍历，NCollection_Map 去重
    TopoDS_Compound allVisible;
    BRep_Builder builder;
    builder.MakeCompound(allVisible);
    NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> seen;
    int rg1EdgeCount = 0, outlineEdgeCount = 0;
    auto addEdges = [&](const TopoDS_Shape& comp, int* counter) {
        if (comp.IsNull())
            return;
        for (TopExp_Explorer eExp(comp, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            if (seen.Add(eExp.Current()))
            {
                builder.Add(allVisible, eExp.Current());
                if (counter) (*counter)++;
            }
        }
    };
    addEdges(visibleEdgesCompound, nullptr);
    addEdges(rg1Edges, &rg1EdgeCount);
    addEdges(outlineEdges, &outlineEdgeCount);
    std::cout << "调试：VCompound有效（含G1相切边" << rg1EdgeCount
              << "条、轮廓边" << outlineEdgeCount << "条），开始遍历边" << std::endl;

    // 边遍历：按曲线类型输出解析形式（线段/圆弧/圆），不离散化。
    // HLR 输出几何已在视图空间，x/y 即 2D 投影坐标，直接取用。
    TopExp_Explorer exp;
    int edgeTotal = 0;
    int lineCount = 0;      // 线段数
    int arcCount = 0;       // 圆弧数
    int circleCount = 0;    // 整圆数
    int skippedCount = 0;   // 跳过数（退化/参数非法）
    int unsupportedCount = 0; // 不支持的曲线类型（椭圆/BSpline 等）
    for (exp.Init(allVisible, TopAbs_EDGE); exp.More(); exp.Next())
    {
        edgeTotal++;
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());

        // 跳过退化边
        if (BRep_Tool::Degenerated(edge))
        {
            skippedCount++;
            continue;
        }

        BRepAdaptor_Curve curveAdaptor(edge);
        GeomAbs_CurveType curveType = curveAdaptor.GetType();

        double firstParam = curveAdaptor.FirstParameter();
        double lastParam = curveAdaptor.LastParameter();
        if (lastParam <= firstParam)
        {
            skippedCount++;
            continue;
        }

        ProjectedCurve2D curve;

        if (curveType == GeomAbs_Line)
        {
            // 线段：取两端顶点
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2);
            if (v1.IsNull() || v2.IsNull())
            {
                skippedCount++;
                continue;
            }
            gp_Pnt p3d_1 = BRep_Tool::Pnt(v1);
            gp_Pnt p3d_2 = BRep_Tool::Pnt(v2);

            curve.type = ProjectedCurve2DType::Line;
            curve.startPoint = gp_Pnt2d(p3d_1.X(), p3d_1.Y());
            curve.endPoint = gp_Pnt2d(p3d_2.X(), p3d_2.Y());
            lineCount++;
        }
        else if (curveType == GeomAbs_Circle)
        {
            // 圆 / 圆弧：解析提取圆心、半径、起止角
            const gp_Circ& circle = curveAdaptor.Circle();
            gp_Pnt center3d = circle.Location();
            double radius = circle.Radius();

            // 方向判断：gp_Circ 的轴方向决定角度旋转方向。
            // 轴正方向与视图 Z 同向时为逆时针（右手定则），反向时为顺时针。
            gp_Dir axisDir = circle.Axis().Direction();
            bool isClockwise = (axisDir.Z() < 0.0);

            curve.center = gp_Pnt2d(center3d.X(), center3d.Y());
            curve.radius = radius;
            curve.startAngle = firstParam;
            curve.endAngle = lastParam;
            curve.isClockwise = isClockwise;

            // 弧两端点
            gp_Pnt pStart = curveAdaptor.Value(firstParam);
            gp_Pnt pEnd = curveAdaptor.Value(lastParam);
            curve.startPoint = gp_Pnt2d(pStart.X(), pStart.Y());
            curve.endPoint = gp_Pnt2d(pEnd.X(), pEnd.Y());

            // 参数范围接近 2π 则为整圆，否则为圆弧
            double paramRange = lastParam - firstParam;
            if (paramRange >= 2.0 * 3.14159265358979323846 - 1.0e-6)
            {
                curve.type = ProjectedCurve2DType::Circle;
                circleCount++;
            }
            else
            {
                curve.type = ProjectedCurve2DType::Arc;
                arcCount++;
            }
        }
        else
        {
            // 椭圆、BSpline 等其他类型暂不支持解析输出，跳过
            unsupportedCount++;
            continue;
        }

        outCurves.push_back(curve);
    }
    std::cout << "调试：VCompound总Edge数量=" << edgeTotal
              << "（线段=" << lineCount << " 圆弧=" << arcCount
              << " 整圆=" << circleCount << " 跳过=" << skippedCount
              << " 不支持=" << unsupportedCount << "）" << std::endl;
    std::cout << "调试：遍历结束，输出解析曲线数量=" << outCurves.size() << std::endl;

    return !outCurves.empty();
}

// 多面体 HLR 投影：BRepMesh 三角化 -> HLRBRep_PolyAlgo 消隐 -> 2D 折线
bool ProjectShapeTo2DByPolyHLR(const std::vector<TopoDS_Shape>& shapes,
    const gp_Dir& projDir,
    const gp_Dir& projUp,
    std::vector<ProjectedPolyline2D>& outPolylines)
{
    outPolylines.clear();
    if (shapes.empty())
    {
        std::cout << "调试[PolyHLR]：输入shape集合为空" << std::endl;
        return false;
    }

    // 权威视图变换（与解析 HLR、脊线投影共用同一变换）
    gp_Trsf trsf;
    if (!BuildViewTransform(projDir, projUp, trsf))
        return false;
    HLRAlgo_Projector projector(trsf, Standard_False, 0.0);

    // 合并为单个 Compound 一次 Load
    TopoDS_Compound allShape;
    BRep_Builder builder;
    builder.MakeCompound(allShape);
    int addCount = 0;
    for (const auto& shape : shapes)
    {
        if (!shape.IsNull())
        {
            builder.Add(allShape, shape);
            addCount++;
        }
    }
    if (addCount == 0)
    {
        std::cout << "调试[PolyHLR]：没有有效shape" << std::endl;
        return false;
    }

    // 三角化（多面体 HLR 的前提）。
    // 线性偏转 0.5mm（杆件长数百、切角特征 2~27mm 均可分辨；平面上的
    // 三角形在平面内，斜切面投影边无逼近误差），角偏转 0.5rad。
    BRepMesh_IncrementalMesh mesher(allShape, 0.5, Standard_False, 0.5, Standard_True);
    mesher.Perform();
    if (!mesher.IsDone())
    {
        std::cout << "调试[PolyHLR]：三角化失败" << std::endl;
        return false;
    }

    // 多面体 HLR：Load -> Update（内部直接完成三角形遮挡分类，无 Hide()）
    Handle(HLRBRep_PolyAlgo) polyAlgo = new HLRBRep_PolyAlgo();
    polyAlgo->Projector(projector);
    polyAlgo->Load(allShape);
    polyAlgo->Update();
    std::cout << "调试[PolyHLR]：Update()完成（多面体消隐）" << std::endl;

    HLRBRep_PolyHLRToShape polyConverter;
    polyConverter.Update(polyAlgo);

    // 与解析流程一致：合并 锐边VCompound + G1相切边Rg1LineVCompound
    // + 轮廓边OutLineVCompound，NCollection_Map 去重
    TopoDS_Shape visibleEdges = polyConverter.VCompound();
    TopoDS_Shape rg1Edges     = polyConverter.Rg1LineVCompound();
    TopoDS_Shape outlineEdges = polyConverter.OutLineVCompound();

    TopoDS_Shape hiddenEdges = polyConverter.HCompound();
    int hiddenCount = 0;
    if (!hiddenEdges.IsNull())
    {
        for (TopExp_Explorer hExp(hiddenEdges, TopAbs_EDGE); hExp.More(); hExp.Next())
            hiddenCount++;
    }
    std::cout << "调试[PolyHLR]：被遮挡隐藏边(HCompound)数量=" << hiddenCount << std::endl;

    if (visibleEdges.IsNull() && rg1Edges.IsNull() && outlineEdges.IsNull())
    {
        std::cout << "调试[PolyHLR]：VCompound/Rg1Line/OutLine 均为NULL，无可见边" << std::endl;
        return false;
    }

    TopoDS_Compound allVisible;
    builder.MakeCompound(allVisible);
    NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher> seen;
    auto addEdges = [&](const TopoDS_Shape& comp) {
        if (comp.IsNull())
            return;
        for (TopExp_Explorer eExp(comp, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            if (seen.Add(eExp.Current()))
                builder.Add(allVisible, eExp.Current());
        }
    };
    addEdges(visibleEdges);
    addEdges(rg1Edges);
    addEdges(outlineEdges);

    // 可见边 -> 2D 折线。几何已在视图空间，x/y 即投影坐标。
    int edgeTotal = 0, lineEdges = 0, sampledEdges = 0, skipped = 0;
    for (TopExp_Explorer exp(allVisible, TopAbs_EDGE); exp.More(); exp.Next())
    {
        edgeTotal++;
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        if (BRep_Tool::Degenerated(edge))
        {
            skipped++;
            continue;
        }

        BRepAdaptor_Curve curve(edge);
        double fp = curve.FirstParameter();
        double lp = curve.LastParameter();
        if (!(lp > fp))
        {
            skipped++;
            continue;
        }

        ProjectedPolyline2D pl;
        if (curve.GetType() == GeomAbs_Line)
        {
            // 多面体边/轮廓多为直线段：取两端点
            gp_Pnt a = curve.Value(fp);
            gp_Pnt b = curve.Value(lp);
            pl.points.push_back(gp_Pnt2d(a.X(), a.Y()));
            pl.points.push_back(gp_Pnt2d(b.X(), b.Y()));
            lineEdges++;
        }
        else
        {
            // 兜底：极少数非直线输出（光滑轮廓重构曲线）按 12 点采样
            const int N = 12;
            for (int k = 0; k <= N; ++k)
            {
                double par = fp + (lp - fp) * k / N;
                gp_Pnt q = curve.Value(par);
                pl.points.push_back(gp_Pnt2d(q.X(), q.Y()));
            }
            sampledEdges++;
        }
        outPolylines.push_back(pl);
    }

    std::cout << "调试[PolyHLR]：可见Edge总数=" << edgeTotal
              << "（直线段=" << lineEdges << " 采样兜底=" << sampledEdges
              << " 跳过=" << skipped << "），输出折线=" << outPolylines.size() << std::endl;

    return !outPolylines.empty();
}

bool IsShapeValid(const TopoDS_Shape& shape)
{
    BRepCheck_Analyzer checker(shape);
    return checker.IsValid();
}

TopoDS_Shape FixShape(const TopoDS_Shape& shape)
{
    Handle(ShapeFix_Shape) fix = new ShapeFix_Shape(shape);
    fix->Perform();
    return fix->Shape();
}

// outerWire:外轮廓闭合wire
// holeWires:多个内孔闭合wire
TopoDS_Face BuildFaceWithHoles(const TopoDS_Wire& outerWire,
                               const std::vector<TopoDS_Wire>& holeWires,
                               const gp_Pnt& planeOrigin,
                               const gp_Dir& planeNormal,
                               Standard_Real prec )
{
    // 1. 创建支撑平面
    Handle(Geom_Plane) hPlane = new Geom_Plane(planeOrigin, planeNormal);

    // 2. ShapeFix_Face初始化：传入曲面+精度
    ShapeFix_Face faceFix;
    faceFix.Init(hPlane, prec); // 正确！7.8支持这个Init重载

    // 3. 添加所有wire：先外环，再所有内环
    faceFix.Add(outerWire);
    for(const auto& w : holeWires)
    {
        faceFix.Add(w);
    }

    // 4. 核心：自动调整所有wire的环绕方向（外环/内环方向一致性）
    faceFix.FixOrientation();

    // 5. 执行修复，生成面
    // 注意：Perform() 的布尔返回值表示“是否执行了修复动作”，
    // 不能直接当作成功标志（面本身合格时也可能返回 false），
    // 因此以结果面是否为空作为成功判据。
    faceFix.Perform();

    // 调试：输出 ShapeFix_Face 的所有状态位（独立判断，打印完整状态组合）
    // ShapeExtend_OK   : 无任何标志位（一切正常，未做修复）
    // ShapeExtend_DONE : 任意 DONEi 被置位（执行了某种修复/构建动作）
    // ShapeExtend_FAIL : 任意 FAILi 被置位（构建失败）
    std::cout << "调试：faceFix 状态位输出：" << std::endl;
    std::cout << "  ShapeExtend_OK    = " << (faceFix.Status(ShapeExtend_OK)   ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE1 = " << (faceFix.Status(ShapeExtend_DONE1) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE2 = " << (faceFix.Status(ShapeExtend_DONE2) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE3 = " << (faceFix.Status(ShapeExtend_DONE3) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE4 = " << (faceFix.Status(ShapeExtend_DONE4) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE5 = " << (faceFix.Status(ShapeExtend_DONE5) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE6 = " << (faceFix.Status(ShapeExtend_DONE6) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE7 = " << (faceFix.Status(ShapeExtend_DONE7) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE8 = " << (faceFix.Status(ShapeExtend_DONE8) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_DONE  = " << (faceFix.Status(ShapeExtend_DONE)  ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL1 = " << (faceFix.Status(ShapeExtend_FAIL1) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL2 = " << (faceFix.Status(ShapeExtend_FAIL2) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL3 = " << (faceFix.Status(ShapeExtend_FAIL3) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL4 = " << (faceFix.Status(ShapeExtend_FAIL4) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL5 = " << (faceFix.Status(ShapeExtend_FAIL5) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL6 = " << (faceFix.Status(ShapeExtend_FAIL6) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL7 = " << (faceFix.Status(ShapeExtend_FAIL7) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL8 = " << (faceFix.Status(ShapeExtend_FAIL8) ? "true" : "false") << std::endl;
    std::cout << "  ShapeExtend_FAIL  = " << (faceFix.Status(ShapeExtend_FAIL)  ? "true" : "false") << std::endl;

    TopoDS_Face resultFace = faceFix.Face();
    if (resultFace.IsNull())
    {
        // 构建失败，检查：wire是否闭合、是否全部落在平面、内环完全在外环内部
        return TopoDS_Face();
    }
    return resultFace;
}

TopoDS_Shape BuildSweepWireframe(const TopoDS_Wire& spineWire1,
                                 const TopoDS_Wire& spineWire2,
                                 const TopoDS_Wire& sectionWire)
{
    if (sectionWire.IsNull())
    {
        std::cout << "错误：截面轮廓为空" << std::endl;
        return TopoDS_Shape();
    }

    // 1. 脊线 + 截面 → BRepOffsetAPI_MakePipeShell 生成扫掠壳
    //    每条脊线独立扫掠一次，两条脊线各生成一个扫掠壳
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    const TopoDS_Wire spineWires[2] = { spineWire1, spineWire2 };
    int shellCount = 0;
    for (int i = 0; i < 2; ++i)
    {
        const TopoDS_Wire& spine = spineWires[i];
        if (spine.IsNull())
        {
            std::cout << "调试：脊线" << (i + 1) << "为空，跳过" << std::endl;
            continue;
        }

        BRepOffsetAPI_MakePipeShell pipeMaker(spine);
        // 转折处用右角过渡（保角），扫掠方向用修正 Frenet 三面体
        // （SetMode(false)=CorrectedFrenet，直线脊线不会因 Frenet 绞扭）
        pipeMaker.SetTransitionMode(BRepBuilderAPI_RightCorner);
        pipeMaker.SetMode(Standard_False);
        // 截面不强制接触脊线、不做位置校正，按原始相对位置扫掠
        pipeMaker.Add(sectionWire, Standard_False, Standard_False);

        if (!pipeMaker.IsReady())
        {
            std::cout << "调试：脊线" << (i + 1) << " PipeShell未就绪（脊线或截面无效），跳过" << std::endl;
            continue;
        }
        pipeMaker.Build();
        if (!pipeMaker.IsDone())
        {
            std::cout << "调试：脊线" << (i + 1) << " PipeShell构建失败，跳过" << std::endl;
            continue;
        }

        TopoDS_Shape sweepShell = pipeMaker.Shape();
        if (sweepShell.IsNull())
        {
            std::cout << "调试：脊线" << (i + 1) << " 扫掠结果为空，跳过" << std::endl;
            continue;
        }
        shellCount++;

        // 2. 提取所有 Edge 组装成 Compound 线框骨架
        int edgeCount = 0;
        for (TopExp_Explorer eExp(sweepShell, TopAbs_EDGE); eExp.More(); eExp.Next())
        {
            builder.Add(compound, eExp.Current());
            edgeCount++;
        }
        std::cout << "调试：脊线" << (i + 1) << " 扫掠完成，提取边数=" << edgeCount << std::endl;
    }

    if (shellCount == 0)
    {
        std::cout << "错误：两条脊线均未生成扫掠壳" << std::endl;
        return TopoDS_Shape();
    }

    // 3. BRepLib::BuildCurves3d 补全 3D 曲线
    //    扫掠产生的边可能只携带 pcurve（参数曲线），缺少 3D 曲线时
    //    HLR 遮挡/投影计算会失败或退化，此处统一补全
    BRepLib::BuildCurves3d(compound);
    std::cout << "调试：BuildCurves3d 补全完成，扫掠壳数量=" << shellCount << std::endl;

    return compound;
}

TopoDS_Shape BuildSweepShell(const TopoDS_Wire& spineWire, const TopoDS_Wire& sectionWire)
{
    if (spineWire.IsNull())
    {
        std::cout << "错误：扫掠脊线为空" << std::endl;
        return TopoDS_Shape();
    }
    if (sectionWire.IsNull())
    {
        std::cout << "错误：截面轮廓为空" << std::endl;
        return TopoDS_Shape();
    }

    // 脊线 + 截面 → BRepOffsetAPI_MakePipeShell 生成带面扫掠壳
    // 参数与 BuildSweepWireframe 保持一致：保角过渡 + 修正 Frenet 三面体
    BRepOffsetAPI_MakePipeShell pipeMaker(spineWire);
    pipeMaker.SetTransitionMode(BRepBuilderAPI_RightCorner);
    pipeMaker.SetMode(Standard_False);
    pipeMaker.Add(sectionWire, Standard_False, Standard_False);

    if (!pipeMaker.IsReady())
    {
        std::cout << "错误：PipeShell未就绪（脊线或截面无效）" << std::endl;
        return TopoDS_Shape();
    }
    pipeMaker.Build();
    if (!pipeMaker.IsDone())
    {
        std::cout << "错误：PipeShell构建失败" << std::endl;
        return TopoDS_Shape();
    }

    TopoDS_Shape shell = pipeMaker.Shape();
    if (shell.IsNull())
    {
        std::cout << "错误：扫掠结果为空" << std::endl;
        return TopoDS_Shape();
    }

    // 补全边的 3D 曲线（HLR 投影依赖）
    BRepLib::BuildCurves3d(shell);

    // 整体拓扑修复：修正面法向不一致（闭合脊线拐角过渡面可能朝内）、
    // 退化面、边的 3D 曲线缺失等问题。HLR 遮挡判定依赖面法向一致性，
    // 法向不一致的面会导致遮挡关系计算错误
    {
        Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(shell);
        fixer->Perform();
        TopoDS_Shape fixed = fixer->Shape();
        if (!fixed.IsNull())
        {
            int faceBefore = 0, faceAfter = 0;
            for (TopExp_Explorer f(shell, TopAbs_FACE); f.More(); f.Next()) faceBefore++;
            for (TopExp_Explorer f(fixed, TopAbs_FACE); f.More(); f.Next()) faceAfter++;
            if (faceAfter > 0)
            {
                std::cout << "调试：ShapeFix_Shape 修复完成，面数 " << faceBefore
                          << "->" << faceAfter << std::endl;
                shell = fixed;
            }
        }
    }

    // ShapeFix_Shape 可能修改边几何（重建 3D 曲线），需重新补全 3D 曲线，
    // 否则 HLR 遮挡/投影计算会因部分边缺失 3D 曲线而退化为可见或丢失（破面）
    BRepLib::BuildCurves3d(shell);

    // ---- 封闭开口（补端盖 / 修复闭合点破口）----
    // PipeShell 只生成侧面。开脊线两端是开口自由边界（需补端盖）；
    // 闭合脊线在闭合点处可能产生破口（开轮廓），也需检测并修复。
    // 步骤：1. ShapeAnalysis_FreeBounds 提取闭合自由边界（端盖）和开自由边界（破口）
    //       2. 对闭合 wire 拟合平面生成端盖面；对开 wire 闭合并补面
    //       3. BRepBuilderAPI_Sewing 缝合成封闭壳
    ShapeAnalysis_FreeBounds freeAnalyzer(shell, Standard_False, Standard_False, Standard_False);
    TopoDS_Compound closedFreeWires = freeAnalyzer.GetClosedWires();
    TopoDS_Compound openFreeWires   = freeAnalyzer.GetOpenWires();

    BRepBuilderAPI_Sewing sewer(1.0e-6);
    sewer.Add(shell);
    int capCount = 0;
    int patchCount = 0;

    // 模型包围盒（脊线沿 +X 时用于判断端盖位于起点端/终点端并确定外法向）
    Bnd_Box shellBox;
    BRepBndLib::Add(shell, shellBox);
    double sxm, sym, szm, sxM, syM, szM;
    shellBox.Get(sxm, sym, szm, sxM, syM, szM);

    // 处理闭合自由边界（端盖）
    for (TopExp_Explorer wExp(closedFreeWires, TopAbs_WIRE); wExp.More(); wExp.Next())
    {
        TopoDS_Wire capWire = TopoDS::Wire(wExp.Current());
        // OnlyPlane=true：由平面 wire 直接拟合支撑平面（扫掠端面保持截面平面形状）
        BRepBuilderAPI_MakeFace faceMaker(capWire, Standard_True);
        if (faceMaker.IsDone() && !faceMaker.Face().IsNull())
        {
            TopoDS_Face capFace = faceMaker.Face();

            // 修正端盖朝向：外法向必须指向壳外侧（起点端 -X，终点端 +X）。
            // MakeFace 拟合的平面法向取决于 wire 方向，可能朝内；
            // 朝内的端盖会造成缝合壳朝向不一致（影响后续实体化/布尔运算）
            Bnd_Box capBox;
            BRepBndLib::Add(capWire, capBox);
            double cxm, cym, czm, cxM, cyM, czM;
            capBox.Get(cxm, cym, czm, cxM, cyM, czM);
            bool startEnd = (fabs(cxm - sxm) + fabs(cxM - sxm) <=
                             fabs(cxm - sxM) + fabs(cxM - sxM));

            BRepAdaptor_Surface capSurf(capFace);
            if (capSurf.GetType() == GeomAbs_Plane)
            {
                gp_Dir n = capSurf.Plane().Axis().Direction();
                if (capFace.Orientation() == TopAbs_REVERSED)
                    n.Reverse();
                double expect = startEnd ? -1.0 : 1.0;
                if (n.X() * expect < 0.0)
                    capFace.Reverse();
            }

            sewer.Add(capFace);
            capCount++;
        }
        else
        {
            std::cout << "警告：端部自由边界无法拟合平面，端盖未生成" << std::endl;
        }
    }

    // 处理开自由边界（闭合点破口）：闭合脊线在闭合点处可能产生破口（开轮廓），
    // 需要先闭合 wire 再补面。破口不补会导致 HLR 遮挡判定失效（内壁边泄露）
    for (TopExp_Explorer wExp(openFreeWires, TopAbs_WIRE); wExp.More(); wExp.Next())
    {
        TopoDS_Wire openWire = TopoDS::Wire(wExp.Current());
        // 用 ShapeFix_Wire 闭合开轮廓：将首尾顶点合并为同一顶点
        ShapeFix_Wire wireFixer;
        wireFixer.Load(openWire);
        wireFixer.FixClosed();
        TopoDS_Wire fixedWire = wireFixer.Wire();
        if (fixedWire.IsNull())
            fixedWire = openWire;   // 闭合失败，用原始 wire 尝试补面

        BRepBuilderAPI_MakeFace faceMaker(fixedWire, Standard_True);
        if (faceMaker.IsDone() && !faceMaker.Face().IsNull())
        {
            sewer.Add(faceMaker.Face());
            patchCount++;
        }
    }
    std::cout << "调试：端盖补面=" << capCount << "，破口修复=" << patchCount << std::endl;

    TopoDS_Shape closed = shell;
    if (capCount > 0 || patchCount > 0)
    {
        sewer.Perform();
        closed = sewer.SewedShape();
    }

    // 整体朝向修正：PipeShell 生成的壳（含闭合脊线）可能整体朝内
    // （外法向指向内侧），导致 HLR 遮挡判定异常（本应被遮挡的边泄露为可见，
    // 本应可见的边被错误隐藏 = 破面+遮挡错误）。
    // 无论是否补了端盖，只要壳是闭合的（有向体积非零），就检查体积符号：
    // 负体积 = 朝内，整体反转使外法向朝外。
    // 开口壳的有向体积 ≈ 0，自动跳过反转。
    {
        GProp_GProps volProps;
        BRepGProp::VolumeProperties(closed, volProps);
        double vol = volProps.Mass();
        if (fabs(vol) > 1.0e-10 && vol < 0.0)
        {
            closed.Reverse();
            std::cout << "调试：扫掠壳有向体积为负(" << vol
                      << ")，已整体反转使外法向朝外" << std::endl;
        }
        else if (fabs(vol) > 1.0e-10)
        {
            std::cout << "调试：扫掠壳有向体积=" << vol << "，朝向正常" << std::endl;
        }
        else
        {
            std::cout << "调试：扫掠壳有向体积≈0（开口壳），跳过朝向检查" << std::endl;
        }
    }

    // 统计面数，确认壳携带面（遮挡判定依赖面）且端盖已补上
    int faceCount = 0;
    for (TopExp_Explorer fExp(closed, TopAbs_FACE); fExp.More(); fExp.Next())
        faceCount++;
    std::cout << "调试：带面扫掠壳生成完成，面数=" << faceCount
              << "（侧面+端盖，两端封闭）" << std::endl;

    return closed;
}
