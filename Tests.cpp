#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include "ProjectHelper.h"
#include "Shape.h"
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <ShapeFix_Shape.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax1.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_TVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <utility>
#include <Standard_Failure.hxx>
#include <Standard_ConstructionError.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <cmath>
#include <fstream>
#include <sstream>
#include <regex>
#include <algorithm>
#include <stdexcept>
#include <cctype>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <nlohmann/json.hpp>

#include "dl_dxf.h"
#include "dl_creationadapter.h"

int OCCT_Test() {
    TopoDS_Solid solid;
    try {
        gp_Pnt p1(-202.000000, 3330.000020, -0.000020);
        gp_Pnt p2(298.000000, 3880.000020, 1999.999980);
        BRepPrimAPI_MakeBox mkBox(p1,100,100,100);
        if (mkBox.IsDone()) {
            solid = mkBox.Solid();
            std::cout << "success" << std::endl;
        }

        TopoDS_Shape tmpShape = mkBox.Shape();
    }
    catch (const Standard_DomainError& e)
    {
        // 捕获这个异常，就能确认是OCCT内部抛出DomainError
        std::cout << "DomainError: " << e.GetMessageString() << std::endl;
    }

    gp_Pnt p1(0, 0, 0);
    gp_Pnt p2(100, 100, 100);

    double xmin = std::min(p1.X(), p2.X());
    double ymin = std::min(p1.Y(), p2.Y());
    double zmin = std::min(p1.Z(), p2.Z());
    double dx = fabs(p2.X() - p1.X());
    double dy = fabs(p2.Y() - p1.Y());
    double dz = fabs(p2.Z() - p1.Z());

    BRepPrimAPI_MakeBox mkBox(gp_Pnt(xmin, ymin, zmin), dx, dy, dz);
    if (!mkBox.IsDone()) {
        std::cout << "makeBox error!!!" << std::endl;
    }

    mkBox.Shape();
    // 1. 创建一个100x100x100的立方体
    TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 1.0e-9), gp_Pnt(100, 100, 100)).Shape();

    // 2. 创建球体，放在立方体中间
    TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(50, 50, 50), 30).Shape();

    // 3. 布尔差集：立方体挖掉球体
    BRepAlgoAPI_Cut cutOp(box, sphere);
    cutOp.Build();

    if (!cutOp.IsDone())
    {
        std::cout << "布尔运算失败!" << std::endl;
        return -1;
    }

    TopoDS_Shape resultShape = cutOp.Shape();

    // 校验拓扑合法性
    BRepCheck_Analyzer checker(resultShape);
    if (checker.IsValid())
    {
        std::cout << "✅ OCCT库加载正常，布尔运算成功，拓扑合法！" << std::endl;
    }
    else
    {
        std::cout << "⚠️ 生成的Shape拓扑存在问题" << std::endl;
    }

    std::cout << "测试完成" << std::endl;
    return 0;
}

// 线段长度过滤阈值（单一来源，模型实际尺寸约 100~150）
static const double DXF_MIN_SEGMENT_LENGTH = 1.0;     // 小于 1 的线段视为退化碎边，忽略
static const double DXF_MAX_SEGMENT_LENGTH = 1000.0;  // 超长线段视为投影伪边/异常边，忽略

// 将投影曲线集以 POLYLINE 实体写入 DXF 文件（每条曲线一个 POLYLINE）
// 过滤：总长度 < DXF_MIN_SEGMENT_LENGTH 的碎边、或含长度 > DXF_MAX_SEGMENT_LENGTH 的异常线段，不写入
bool WriteProjectedCurvesToDXF(const std::vector<ProjectedPolyline2D>& curves, const char* file) {
    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out(file, DL_Codes::AC1015);
    if (dw == NULL) {
        std::cout << "无法创建 DXF 文件: " << file << std::endl;
        return false;
    }

    dxf.writeHeader(*dw);
    dw->sectionEnd();

    dw->sectionTables();
    dxf.writeVPort(*dw);

    dw->tableLinetypes(1);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dw->tableEnd();

    dw->tableLayers(1);
    dxf.writeLayer(*dw, DL_LayerData("0", 0),
                   DL_Attributes(std::string(""), DL_Codes::white, 100, "CONTINUOUS", 1.0));
    dw->tableEnd();

    dw->tableStyle(1);
    dxf.writeStyle(*dw, DL_StyleData("standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "txt", ""));
    dw->tableEnd();

    dxf.writeView(*dw);
    dxf.writeUcs(*dw);

    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    dxf.writeDimStyle(*dw, 1, 1, 1, 1, 1);

    dxf.writeBlockRecord(*dw);
    dw->tableEnd();
    dw->sectionEnd();

    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    // 实体段：逐条写入 POLYLINE（按长度过滤碎边/超长异常边）
    dw->sectionEntities();
    int polylineCount = 0;
    int skippedShort = 0;   // 总长度 < 最小阈值
    int skippedLong = 0;    // 含超长线段
    for (const auto& curve : curves) {
        if (curve.points.size() < 2)
        {
            continue;
        }

        // 计算该 POLYLINE 的总长度与最长分段长度
        double totalLen = 0.0;
        double maxSegLen = 0.0;
        for (size_t i = 1; i < curve.points.size(); ++i)
        {
            double segLen = curve.points[i - 1].Distance(curve.points[i]);
            totalLen += segLen;
            if (segLen > maxSegLen) maxSegLen = segLen;
        }

        if (totalLen < DXF_MIN_SEGMENT_LENGTH)
        {
            skippedShort++;
            continue;
        }
        if (maxSegLen > DXF_MAX_SEGMENT_LENGTH)
        {
            skippedLong++;
            continue;
        }

        dxf.writePolyline(*dw,
                          DL_PolylineData((int)curve.points.size(), 0, 0, 0),
                          DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
        for (const auto& pt : curve.points)
        {
            dxf.writeVertex(*dw, DL_VertexData(pt.X(), pt.Y(), 0.0));
        }
        dxf.writePolylineEnd(*dw);
        polylineCount++;
    }
    dw->sectionEnd();

    dxf.writeObjects(*dw);
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;

    std::cout << "已写入 " << polylineCount << " 条 POLYLINE 到 DXF 文件: " << file
              << "（过滤：碎边<" << DXF_MIN_SEGMENT_LENGTH << " 共 " << skippedShort
              << " 条，超长边>" << DXF_MAX_SEGMENT_LENGTH << " 共 " << skippedLong << " 条）" << std::endl;
    return true;
}

// 遮挡消隐验证：立方体打一个竖直通孔，等轴测视角下孔底圆边必然被自身遮挡
// （视线先碰到立方体顶面/侧面），该圆边应被 HLR 判定为隐藏边（HCompound）
int OcclusionDemo() {
    std::cout << "\n--- 遮挡消隐演示（带通孔的立方体，孔底圆边应被遮挡）---" << std::endl;
    std::vector<TopoDS_Shape> shapes;
    TopoDS_Shape block = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(100, 100, 100)).Shape();
    TopoDS_Shape tool = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(50, 50, -20), gp_Dir(0, 0, 1)), 25, 140).Shape();
    BRepAlgoAPI_Cut cut(block, tool);
    cut.Build();
    shapes.emplace_back(cut.Shape());

    std::vector<ProjectedPolyline2D> curves;
    bool ok = ProjectBoxSetTo2DCurves(shapes, gp_Dir(1, 1, 1), gp_Dir(1, 0, 0), curves);
    std::cout << "遮挡演示完成，可见轮廓线段数=" << curves.size()
              << "（被遮挡的隐藏边数量见上方 HCompound 调试输出，>0 即说明遮挡消隐生效）" << std::endl;
    return ok ? 0 : -1;
}

// 带孔棱柱生成测试：
// 1) 构造矩形外环 + 圆形内环（孔）；
// 2) 调用 BuildFaceWithHoles 生成带孔平面（方向由 ShapeFix_Face 自动修正）；
// 3) 用 BRepPrimAPI_MakePrism 将带孔面沿法向拉伸成棱柱（孔为通孔）；
// 4) BRepCheck_Analyzer 校验拓扑合法性。
int BuildHollowPrismTest() {
    std::cout << "\n--- 带孔棱柱测试（BuildFaceWithHoles + MakePrism）---" << std::endl;

    // 1. 外环：100 x 80 矩形，位于 z=0 平面
    BRepBuilderAPI_MakePolygon polyMaker;
    polyMaker.Add(gp_Pnt(0,   0,  0));
    polyMaker.Add(gp_Pnt(100, 0,  0));
    polyMaker.Add(gp_Pnt(100, 80, 0));
    polyMaker.Add(gp_Pnt(0,   80, 0));
    polyMaker.Close();
    if (!polyMaker.IsDone())
    {
        std::cout << "外环构造失败" << std::endl;
        return -1;
    }
    TopoDS_Wire outerWire = polyMaker.Wire();

    // 2. 内环：半径 15 的圆孔，圆心 (50, 40, 0)，整圆
    gp_Circ holeCircle(gp_Ax2(gp_Pnt(50, 40, 0), gp_Dir(0, 0, 1)), 15.0);
    BRepBuilderAPI_MakeEdge holeEdgeMaker(holeCircle);
    if (!holeEdgeMaker.IsDone())
    {
        std::cout << "圆孔边构造失败" << std::endl;
        return -1;
    }
    BRepBuilderAPI_MakeWire holeWireMaker(holeEdgeMaker.Edge());
    if (!holeWireMaker.IsDone())
    {
        std::cout << "内环wire构造失败" << std::endl;
        return -1;
    }
    TopoDS_Wire holeWire = holeWireMaker.Wire();

    std::vector<TopoDS_Wire> holeWires;
    holeWires.push_back(holeWire);

    // 3. 调用 BuildFaceWithHoles 生成带孔面（支撑平面过原点，法向 +Z）
    TopoDS_Face baseFace = BuildFaceWithHoles(outerWire, holeWires,
                                              gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), 1.0e-6);
    if (baseFace.IsNull())
    {
        std::cout << "BuildFaceWithHoles 失败：返回空面" << std::endl;
        return -1;
    }

    // 4. 将带孔面沿 +Z 方向拉伸 30，生成带通孔的棱柱
    BRepPrimAPI_MakePrism prismMaker(baseFace, gp_Vec(0, 0, 30));
    if (!prismMaker.IsDone())
    {
        std::cout << "MakePrism 拉伸失败" << std::endl;
        return -1;
    }
    TopoDS_Shape prism = prismMaker.Shape();

    // 5. 拓扑合法性校验
    BRepCheck_Analyzer checker(prism);
    if (checker.IsValid())
    {
        std::cout << "带孔棱柱生成成功（100x80x30，Φ30 通孔），拓扑合法！" << std::endl;
        return 0;
    }

    std::cout << "带孔棱柱拓扑存在问题" << std::endl;
    return -1;
}

// 倾斜平面带孔面测试（真实数据）：
// 轮廓法向为 (12.727922, -12.727922, 0)（XY 平面内 45度对角，单位 (1/√2,-1/√2,0)），
// 所有点满足 x - y = -6571.727874；外轮廓为矩形，内轮廓为矩形孔。
// 验证 BuildFaceWithHoles 对任意倾斜平面（非轴对齐）的支持。
int BuildFaceOnInclinedPlaneTest() {
    std::cout << "\n--- 倾斜平面带孔面测试（真实数据，法向 12.728,-12.728,0）---" << std::endl;

    // 1. 外轮廓（4 段，按给定 Segment 顺序）
    BRepBuilderAPI_MakePolygon outerPoly;
    outerPoly.Add(gp_Pnt(-3094.040984, 3477.686890,   59.999980));
    outerPoly.Add(gp_Pnt(-3477.999966, 3093.727908,   59.999980));
    outerPoly.Add(gp_Pnt(-3477.999966, 3093.727908, 1996.999980));
    outerPoly.Add(gp_Pnt(-3094.040984, 3477.686890, 1996.999980));
    outerPoly.Close();
    if (!outerPoly.IsDone())
    {
        std::cout << "外轮廓构造失败" << std::endl;
        return -1;
    }
    TopoDS_Wire outerWire = outerPoly.Wire();
    std::cout << "外轮廓wire构造完成" << std::endl;

    // 2. 内轮廓（4 段矩形孔）
    BRepBuilderAPI_MakePolygon innerPoly;
    innerPoly.Add(gp_Pnt(-3380.944034, 3190.783840, 1426.543992));
    innerPoly.Add(gp_Pnt(-3208.518325, 3363.209549, 1426.543992));
    innerPoly.Add(gp_Pnt(-3208.518325, 3363.209549, 1075.117751));
    innerPoly.Add(gp_Pnt(-3380.944034, 3190.783840, 1075.117751));
    innerPoly.Close();
    if (!innerPoly.IsDone())
    {
        std::cout << "内轮廓构造失败" << std::endl;
        return -1;
    }
    TopoDS_Wire innerWire = innerPoly.Wire();
    std::cout << "内轮廓wire构造完成" << std::endl;

    std::vector<TopoDS_Wire> holeWires;
    holeWires.push_back(innerWire);

    // 3. 调用 BuildFaceWithHoles：支撑平面取外轮廓起点，法向为给定轮廓方向
    //    gp_Dir 会自动归一化 (12.727922, -12.727922, 0) -> (0.7071, -0.7071, 0)
    TopoDS_Face face = BuildFaceWithHoles(outerWire, holeWires,
                                          gp_Pnt(-3094.040984, 3477.686890, 59.999980),
                                          gp_Dir(12.727922, -12.727922, 0.0), 1.0e-6);
    if (face.IsNull())
    {
        std::cout << "BuildFaceWithHoles 失败：返回空面" << std::endl;
        return -1;
    }

    // 4. 拓扑合法性校验
    BRepCheck_Analyzer checker(face);
    if (checker.IsValid())
    {
        std::cout << "倾斜平面带孔面生成成功，拓扑合法！" << std::endl;
        return 0;
    }

    std::cout << "倾斜平面带孔面已生成但拓扑存在问题" << std::endl;
    return -1;
}


// OCCT 多方向投影测试：
// i_viewDir 视线方向（相机位于该方向一侧，Z 轴），i_viewUp 屏幕上方向（Y 轴），
// offsetx 将本组 2D 轮廓沿屏幕 X 方向平移（多个视图在同一 DXF 中左右排开），
// 结果累积到 outCurves。
int OCCT_ProjectTest(const gp_Dir& i_viewDir, const gp_Dir& i_viewUp, double offsetx, std::vector<ProjectedPolyline2D>& outCurves) {

	std::cout << "\n--- 投影: 视线(" << i_viewDir.X() << "," << i_viewDir.Y() << "," << i_viewDir.Z()
		<< ") 上方向(" << i_viewUp.X() << "," << i_viewUp.Y() << "," << i_viewUp.Z()
		<< ") X偏移=" << offsetx << std::endl;

	// 构造两个Box实体测试
	std::vector<TopoDS_Shape> boxes;
	boxes.emplace_back(BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), gp_Pnt(100, 100, 50)).Shape());
	boxes.emplace_back(BRepPrimAPI_MakeBox(gp_Pnt(30, 30, 50), gp_Pnt(80, 80, 100)).Shape());

	std::vector<ProjectedPolyline2D> curves;

	bool ok = ProjectBoxSetTo2DCurves(boxes, i_viewDir, i_viewUp, curves);
	if (!ok)
	{
		std::cout << "投影计算失败！" << std::endl;
		return -1;
	}

	// 在 2D 屏幕坐标中沿 X 方向平移本组视图（不能平移世界坐标：world-X 在不同
	// 视图中映射到不同屏幕轴，布局不可控；且 for(auto tmp) 是拷贝遍历，Move 不生效）
	double minX = 1e30, minY = 1e30, maxX = -1e30, maxY = -1e30;
	for (auto& curve : curves)
	{
		for (auto& pt : curve.points)
		{
			double x = pt.X() + offsetx;
			double y = pt.Y();
			pt = gp_Pnt2d(x, y);
			if (x < minX) minX = x;
			if (x > maxX) maxX = x;
			if (y < minY) minY = y;
			if (y > maxY) maxY = y;
		}
	}

	std::cout << "输出投影轮廓线段数量：" << curves.size()
		<< "，2D范围 X[" << minX << ", " << maxX << "] Y[" << minY << ", " << maxY << "]" << std::endl;

	outCurves.insert(outCurves.end(), curves.begin(), curves.end());
	return 0;
}
// dxflib 测试用的读取回调：统计读到的 LINE 实体
class DXFReadCounter : public DL_CreationAdapter {
public:
    int lineCount = 0;

    void addLine(const DL_LineData& data) override {
        std::cout << "  读到线段: (" << data.x1 << ", " << data.y1 << ", " << data.z1
                  << ") -> (" << data.x2 << ", " << data.y2 << ", " << data.z2 << ")" << std::endl;
        lineCount++;
    }
};

// dxflib 导入测试：先用 dll 写一个简单 DXF 文件，再用 dll 读回并校验
int DXFLIB_Test() {
    const char* file = "dxflib_test.dxf";

    // ---- 1. 写 DXF（AC1015 = AutoCAD 2000 格式）----
    DL_Dxf dxfWriter;
    DL_WriterA* dw = dxfWriter.out(file, DL_Codes::AC1015);
    if (dw == NULL) {
        std::cout << "dxflib 测试失败: 无法创建 DXF 文件" << std::endl;
        return -1;
    }

    dxfWriter.writeHeader(*dw);
    dw->sectionEnd();

    dw->sectionTables();
    dxfWriter.writeVPort(*dw);

    dw->tableLinetypes(1);
    dxfWriter.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dw->tableEnd();

    dw->tableLayers(1);
    dxfWriter.writeLayer(*dw, DL_LayerData("0", 0),
                         DL_Attributes(std::string(""), DL_Codes::white, 100, "CONTINUOUS", 1.0));
    dw->tableEnd();

    dw->tableStyle(1);
    dxfWriter.writeStyle(*dw, DL_StyleData("standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "txt", ""));
    dw->tableEnd();

    dxfWriter.writeView(*dw);
    dxfWriter.writeUcs(*dw);

    dw->tableAppid(1);
    dxfWriter.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    dxfWriter.writeDimStyle(*dw, 1, 1, 1, 1, 1);

    dxfWriter.writeBlockRecord(*dw);
    dw->tableEnd();
    dw->sectionEnd();

    dw->sectionBlocks();
    dxfWriter.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxfWriter.writeEndBlock(*dw, "*Model_Space");
    dxfWriter.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxfWriter.writeEndBlock(*dw, "*Paper_Space");
    dxfWriter.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxfWriter.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    dw->sectionEntities();
    dxfWriter.writeLine(*dw,
                        DL_LineData(0.0, 0.0, 0.0,     // 起点
                                    100.0, 50.0, 0.0), // 终点
                        DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
    dw->sectionEnd();

    dxfWriter.writeObjects(*dw);
    dxfWriter.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;

    // ---- 2. 读回并校验 ----
    DXFReadCounter counter;
    DL_Dxf dxfReader;
    if (!dxfReader.in(file, &counter)) {
        std::cout << "dxflib 测试失败: 无法读取 DXF 文件" << std::endl;
        return -1;
    }

    if (counter.lineCount == 1) {
        std::cout << "dxflib 测试通过: dll 写入并读回成功, 共读到 "
                  << counter.lineCount << " 条线段" << std::endl;
        return 0;
    }

    std::cout << "dxflib 测试失败: 期望读到 1 条线段, 实际读到 "
              << counter.lineCount << " 条" << std::endl;
    return -1;
}

// ==================== U型扫掠轴测图 DXF 测试 ====================
// 流程：输入轨迹/截面数据 -> 参数验证 -> 构造 wire -> BuildSweepWireframe 扫掠建模
//       -> 等轴测（或指定角度）轴测投影（解析线段/圆弧/圆） -> 写 DXF 文件

static const double AXO_PI = 3.14159265358979323846;

// 扫掠轨迹段：直线（起终点）或圆弧（圆心/半径/法向轴 + 端点）
struct SweepPathSeg
{
    bool isArc;
    gp_Pnt start;
    gp_Pnt end;
    // 圆弧专属参数：圆心、半径、平面法向（start->end 沿该轴逆时针）
    gp_Pnt arcCenter;
    double arcRadius;
    gp_Dir arcAxis;
};

// U型扫掠轨迹数据：顺序相连的线段/圆弧集合
struct SweepPathData
{
    std::vector<SweepPathSeg> segments;
};

// 截面轮廓段（在截面平面局部 UV 坐标中定义）：
// 直线（两端点 + 长度校验值）或圆弧（局部圆心/半径/起止角，相对 U 轴）
struct SectionSeg
{
    bool isArc;
    // 直线段
    gp_Pnt2d lineStart;
    gp_Pnt2d lineEnd;
    double lineLength;          // 直线段长度（一致性校验用）
    // 圆弧段
    gp_Pnt2d arcCenter;
    double arcRadius;
    double startAngle;          // 弧度（标准数学角，相对 U 轴）
    double endAngle;            // 弧度
    bool isClockwise = false;   // 圆弧方向：false=逆时针 startAngle->endAngle，true=顺时针
    // 椭圆段（isEllipse=true 时生效，作为曲线段的一种：isArc 亦为 true）
    // arcCenter=中心，startAngle/endAngle=参数角（弧度，相对长轴方向），
    // isClockwise=轮廓走向；DXF ELLIPSE 参数角恒为逆时针，输出时顺时针段交换起止参数
    bool isEllipse = false;
    double majorRadius = 0.0;    // 长半轴
    double minorRatio = 1.0;     // 短轴/长轴 比
    double majorAxisAngle = 0.0; // 长轴方向角（弧度，相对 U 轴，逆时针）
};

// 带圆弧的截面轮廓数据：平面方位 + 局部轮廓段集合
struct SectionProfileData
{
    gp_Pnt planeOrigin;         // 局部 UV 原点（3D）
    gp_Dir planeNormal;         // 截面平面法向
    gp_Dir planeXRef;           // 局部 U 轴方向（须与法向不平行，V = 法向 x U）
    std::vector<SectionSeg> segments;
};

// 轨迹数据参数验证：段数、连续性、段几何有效性
static bool ValidateSweepPath(const SweepPathData& path)
{
    const double tol = 1.0e-6;
    if (path.segments.size() < 1)
    {
        std::cout << "参数错误：扫掠轨迹至少需要1个线段，当前=" << path.segments.size() << std::endl;
        return false;
    }
    for (size_t i = 0; i < path.segments.size(); ++i)
    {
        const SweepPathSeg& seg = path.segments[i];
        if (seg.isArc)
        {
            if (seg.arcRadius <= tol)
            {
                std::cout << "参数错误：轨迹圆弧段" << (i + 1) << " 半径非法(" << seg.arcRadius << ")" << std::endl;
                return false;
            }
            double r1 = seg.start.Distance(seg.arcCenter);
            double r2 = seg.end.Distance(seg.arcCenter);
            if (fabs(r1 - seg.arcRadius) > 1.0e-4 || fabs(r2 - seg.arcRadius) > 1.0e-4)
            {
                std::cout << "参数错误：轨迹圆弧段" << (i + 1) << " 端点到圆心距离(" << r1 << "/" << r2
                          << ")与半径(" << seg.arcRadius << ")不符" << std::endl;
                return false;
            }
            if (seg.start.Distance(seg.end) < tol)
            {
                std::cout << "参数错误：轨迹圆弧段" << (i + 1) << " 起终点重合" << std::endl;
                return false;
            }
        }
        else
        {
            if (seg.start.Distance(seg.end) < tol)
            {
                std::cout << "参数错误：轨迹直线段" << (i + 1) << " 起终点重合" << std::endl;
                return false;
            }
        }
        if (i > 0 && path.segments[i - 1].end.Distance(seg.start) > tol)
        {
            std::cout << "参数错误：轨迹段" << i << "->" << (i + 1) << " 端点不连续" << std::endl;
            return false;
        }
    }
    return true;
}

// 乱序轨迹段重排：将输入轨迹段（允许顺序混乱、方向相反）重排为首尾相连的有序链，
// 供 ValidateSweepPath / BuildWireFromSweepPath 使用。
// 规则：
//   1. 端点匹配容差 1e-4（与截面顶点容差一致，兼容真实测量数据的微米级间隙）；
//   2. 允许反转段：直线交换起终点；圆弧交换起终点并反转圆弧轴（保证 start->end
//      沿新轴仍为逆时针）；
//   3. 从首段开始向链首/链尾双向贪心生长；所有段必须全部入链，未入链段逐条报告，
//      绝不静默丢弃（不丢边原则）；
//   4. 链首起点≈链尾终点时为闭合轮廓，否则为开口轮廓。
static bool OrderSweepPathSegments(const SweepPathData& input, SweepPathData& ordered)
{
    const double linkTol = 1.0e-4;
    ordered.segments.clear();
    if (input.segments.empty())
    {
        std::cout << "参数错误：轨迹段为空，无法重排" << std::endl;
        return false;
    }

    const size_t n = input.segments.size();
    std::vector<bool> used(n, false);

    // 段的反向副本（见规则2）
    auto reverseSeg = [](const SweepPathSeg& s) {
        SweepPathSeg r = s;
        r.start = s.end;
        r.end   = s.start;
        if (r.isArc)
            r.arcAxis = s.arcAxis.Reversed();
        return r;
    };

    // 有序链（双向生长）：链首起点=front.start，链尾终点=back.end
    std::deque<SweepPathSeg> chain;
    chain.push_back(input.segments[0]);
    used[0] = true;

    bool grew = true;
    while (grew && chain.size() < n)
    {
        grew = false;
        for (size_t i = 0; i < n; ++i)
        {
            if (used[i])
                continue;
            const SweepPathSeg& s = input.segments[i];
            const gp_Pnt& headPnt = chain.front().start;
            const gp_Pnt& tailPnt = chain.back().end;

            // 链尾追加：原向（s.start≈tail）或反向（s.end≈tail）
            if (s.start.Distance(tailPnt) <= linkTol)
            {
                chain.push_back(s);
            }
            else if (s.end.Distance(tailPnt) <= linkTol)
            {
                chain.push_back(reverseSeg(s));
            }
            // 链首前插：原向（s.end≈head）或反向（s.start≈head）
            else if (s.end.Distance(headPnt) <= linkTol)
            {
                chain.push_front(s);
            }
            else if (s.start.Distance(headPnt) <= linkTol)
            {
                chain.push_front(reverseSeg(s));
            }
            else
            {
                continue;
            }
            used[i] = true;
            grew = true;
            break;
        }
    }

    if (chain.size() != n)
    {
        for (size_t i = 0; i < n; ++i)
        {
            if (used[i])
                continue;
            const SweepPathSeg& s = input.segments[i];
            std::cout << "参数错误：轨迹段" << (i + 1)
                      << " 无法连接到已有轮廓链（"
                      << (s.isArc ? "圆弧" : "直线")
                      << " start:" << s.start.X() << "," << s.start.Y() << "," << s.start.Z()
                      << "; end:"   << s.end.X()   << "," << s.end.Y()   << "," << s.end.Z()
                      << ";）" << std::endl;
        }
        std::cout << "参数错误：仅 " << chain.size() << "/" << n
                  << " 个轨迹段完成连接，存在无法连接的段，终止处理（不丢边原则）" << std::endl;
        return false;
    }

    ordered.segments.assign(chain.begin(), chain.end());
    const gp_Pnt& chainHead = ordered.segments.front().start;
    const gp_Pnt& chainTail = ordered.segments.back().end;
    bool isClosed = chainHead.Distance(chainTail) <= linkTol;
    std::cout << "轨迹段重排完成：" << n << " 段全部入链（无丢边），"
              << (isClosed ? "闭合轮廓" : "开口轮廓")
              << "，链首(" << chainHead.X() << "," << chainHead.Y() << "," << chainHead.Z()
              << ") 链尾(" << chainTail.X() << "," << chainTail.Y() << "," << chainTail.Z()
              << ")" << std::endl;
    return true;
}

// 截面数据参数验证：段数、平面方位、连续性、闭合性、段几何有效性
static bool ValidateSectionProfile(const SectionProfileData& section)
{
    const double tol = 1.0e-6;
    if (section.segments.size() < 2)
    {
        std::cout << "参数错误：截面轮廓至少需要2个线段，当前=" << section.segments.size() << std::endl;
        return false;
    }
    if (fabs(section.planeNormal.Dot(section.planeXRef)) > 0.999999)
    {
        std::cout << "参数错误：截面平面法向与U轴参考方向平行" << std::endl;
        return false;
    }
    for (size_t i = 0; i < section.segments.size(); ++i)
    {
        const SectionSeg& seg = section.segments[i];
        if (seg.isArc)
        {
            if (seg.arcRadius <= tol)
            {
                std::cout << "参数错误：截面圆弧段" << (i + 1) << " 半径非法(" << seg.arcRadius << ")" << std::endl;
                return false;
            }
            // 角度跨度方向感知：逆时针 endAngle>startAngle，顺时针 startAngle>endAngle
            double span = seg.isClockwise ? (seg.startAngle - seg.endAngle)
                                          : (seg.endAngle - seg.startAngle);
            if (span <= tol || span > 2.0 * AXO_PI + 1.0e-9)
            {
                std::cout << "参数错误：截面圆弧段" << (i + 1) << " 角度范围非法("
                          << seg.startAngle << " -> " << seg.endAngle
                          << (seg.isClockwise ? "，顺时针)" : "，逆时针)") << std::endl;
                return false;
            }
        }
        else
        {
            if (seg.lineLength <= tol)
            {
                std::cout << "参数错误：截面直线段" << (i + 1) << " 长度非法(" << seg.lineLength << ")" << std::endl;
                return false;
            }
            double actualLen = seg.lineStart.Distance(seg.lineEnd);
            if (fabs(actualLen - seg.lineLength) > 1.0e-4)
            {
                std::cout << "参数错误：截面直线段" << (i + 1) << " 声明长度(" << seg.lineLength
                          << ")与端点距离(" << actualLen << ")不符" << std::endl;
                return false;
            }
        }
    }
    // 圆弧段复用 lineStart/lineEnd 承载起终点，统一做连续性与参数一致性检查
    auto segStart = [&section](size_t i) -> gp_Pnt2d {
        const SectionSeg& s = section.segments[i];
        return s.isArc ? gp_Pnt2d(s.lineStart.X(), s.lineStart.Y()) : s.lineStart;
    };
    auto segEnd = [&section](size_t i) -> gp_Pnt2d {
        const SectionSeg& s = section.segments[i];
        return s.isArc ? gp_Pnt2d(s.lineEnd.X(), s.lineEnd.Y()) : s.lineEnd;
    };
    for (size_t i = 0; i < section.segments.size(); ++i)
    {
        if (section.segments[i].isArc)
        {
            const SectionSeg& arc = section.segments[i];
            gp_Pnt2d expectStart(arc.arcCenter.X() + arc.arcRadius * cos(arc.startAngle),
                                 arc.arcCenter.Y() + arc.arcRadius * sin(arc.startAngle));
            gp_Pnt2d expectEnd(arc.arcCenter.X() + arc.arcRadius * cos(arc.endAngle),
                               arc.arcCenter.Y() + arc.arcRadius * sin(arc.endAngle));
            if (segStart(i).Distance(expectStart) > 1.0e-4)
            {
                std::cout << "参数错误：截面圆弧段" << (i + 1) << " 起点 lineStart 与圆弧参数起点不符" << std::endl;
                return false;
            }
            if (segEnd(i).Distance(expectEnd) > 1.0e-4)
            {
                std::cout << "参数错误：截面圆弧段" << (i + 1) << " 终点 lineEnd 与圆弧参数终点不符" << std::endl;
                return false;
            }
        }
        if (i > 0 && segEnd(i - 1).Distance(segStart(i)) > tol)
        {
            std::cout << "参数错误：截面段" << i << "->" << (i + 1) << " 端点不连续" << std::endl;
            return false;
        }
    }
    if (segEnd(section.segments.size() - 1).Distance(segStart(0)) > tol)
    {
        std::cout << "参数错误：截面轮廓不闭合（末段终点与首段起点距离>" << tol << "）" << std::endl;
        return false;
    }
    return true;
}

// 轨迹数据 -> TopoDS_Wire（直线段直接两点成边；圆弧由圆心/轴+端点反算参数角）
static TopoDS_Wire BuildWireFromSweepPath(const SweepPathData& path)
{
    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 0; i < path.segments.size(); ++i)
    {
        const SweepPathSeg& seg = path.segments[i];
        TopoDS_Edge edge;
        if (seg.isArc)
        {
            gp_Ax2 circAx(seg.arcCenter, seg.arcAxis);
            gp_Circ circ(circAx, seg.arcRadius);
            // 在 Ax2 默认参数平面内由端点反算起止角（逆时针归一化）
            gp_Dir xDir = circAx.XDirection();
            gp_Dir yDir = circAx.YDirection();
            gp_Vec v1(seg.arcCenter, seg.start);
            gp_Vec v2(seg.arcCenter, seg.end);
            double a1 = atan2(v1.Dot(yDir), v1.Dot(xDir));
            double a2 = atan2(v2.Dot(yDir), v2.Dot(xDir));
            if (a2 <= a1) a2 += 2.0 * AXO_PI;
            GC_MakeArcOfCircle arcMaker(circ, a1, a2, Standard_True);
            if (!arcMaker.IsDone())
            {
                std::cout << "几何错误：轨迹圆弧段" << (i + 1) << " 构造失败" << std::endl;
                return TopoDS_Wire();
            }
            BRepBuilderAPI_MakeEdge edgeMaker(arcMaker.Value());
            if (!edgeMaker.IsDone())
            {
                std::cout << "几何错误：轨迹圆弧段" << (i + 1) << " 边构造失败" << std::endl;
                return TopoDS_Wire();
            }
            edge = edgeMaker.Edge();
        }
        else
        {
            BRepBuilderAPI_MakeEdge edgeMaker(seg.start, seg.end);
            if (!edgeMaker.IsDone())
            {
                std::cout << "几何错误：轨迹直线段" << (i + 1) << " 边构造失败" << std::endl;
                return TopoDS_Wire();
            }
            edge = edgeMaker.Edge();
        }
        wireMaker.Add(edge);
        if (!wireMaker.IsDone())
        {
            std::cout << "几何错误：轨迹wire组装失败（段" << (i + 1) << "）" << std::endl;
            return TopoDS_Wire();
        }
    }
    // 闭合脊线检测：末段终点与首段起点重合时，确保 wire 被标记为闭合，
    // 使 PipeShell 在闭合点处正确闭合截面（否则闭合点处截面不连续→破面）
    TopoDS_Wire result = wireMaker.Wire();
    if (path.segments.size() >= 2)
    {
        const gp_Pnt& first = path.segments.front().start;
        const gp_Pnt& last  = path.segments.back().end;
        if (first.Distance(last) < 1.0e-6 && !result.Closed())
        {
            // BRepBuilderAPI_MakeWire 未自动标记闭合，用 BRep_Builder 手动设置
            result.Closed(Standard_True);
        }
    }
    return result;
}

// 截面数据 -> TopoDS_Wire（局部 UV -> 截面平面 3D）
// 逆时针圆弧按起止角构造；顺时针圆弧反转圆轴后按参数角构造（轴反向后参数
// 递增方向在世界视角下即顺时针）。
// 关键：相邻段共享同一个 TopoDS_Vertex 实例，末段终点与首段起点共用首顶点。
// 真实测量数据仅6位小数，圆弧端点投影到理论圆存在~1e-6间隙（超过默认连接容差
// 1e-7），共享顶点可保证拓扑必然连通；边曲线由参数角trim，几何仍取理论圆弧。
// 约定：圆弧段的起终点也存放在 lineStart / lineEnd 字段中
static TopoDS_Wire BuildWireFromSection(const SectionProfileData& section)
{
    gp_Ax2 planeAx(section.planeOrigin, section.planeNormal, section.planeXRef);
    gp_XYZ xu = planeAx.XDirection().XYZ();
    gp_XYZ yv = planeAx.YDirection().XYZ();
    gp_XYZ org = section.planeOrigin.XYZ();
    auto to3D = [&](const gp_Pnt2d& p) {
        return gp_Pnt(org + xu * p.X() + yv * p.Y());
    };

    const size_t n = section.segments.size();

    // 预建连接顶点：verts[i] 为第 i 段起点，verts[n] 复用首顶点实现闭合。
    // 顶点容差取 1e-4：测量数据6位小数，圆弧端点到理论圆存在~1e-6偏差，
    // 默认容差1e-7会使边构造的点-曲线距离校验失败。
    // BRepBuilderAPI_MakeVertex 无容差构造，通过 BRep_TVertex::Tolerance(T) 设置
    const double vertexTol = 1.0e-4;
    auto makeVertexTol = [&](const gp_Pnt& p) {
        TopoDS_Vertex v = BRepBuilderAPI_MakeVertex(p);
        static_cast<BRep_TVertex*>(v.TShape().get())->Tolerance(vertexTol);
        return v;
    };
    std::vector<TopoDS_Vertex> verts(n + 1);
    for (size_t i = 0; i < n; ++i)
        verts[i] = makeVertexTol(to3D(section.segments[i].lineStart));
    verts[n] = verts[0];

    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 0; i < n; ++i)
    {
        const SectionSeg& seg = section.segments[i];
        TopoDS_Edge edge;
        if (seg.isArc)
        {
            // 圆轴：顺时针弧反转法向，使其参数递增方向沿弧的实际走向
            gp_Dir axisNormal = seg.isClockwise ? section.planeNormal.Reversed()
                                                : section.planeNormal;
            gp_Circ circ(gp_Ax2(to3D(seg.arcCenter), axisNormal, section.planeXRef),
                         seg.arcRadius);

            double alpha1 = seg.startAngle;
            double alpha2 = seg.endAngle;
            if (seg.isClockwise)
            {
                // 反转轴圆参数 alpha' = -alpha（mod 2PI），换算为递增参数区间
                alpha1 = -seg.startAngle;
                while (alpha1 < 0.0)            alpha1 += 2.0 * AXO_PI;
                while (alpha1 >= 2.0 * AXO_PI)  alpha1 -= 2.0 * AXO_PI;
                alpha2 = alpha1 + (seg.startAngle - seg.endAngle);  // 顺时针跨度>0
            }

            // 显式参数界决定trim区间，共享顶点仅承担拓扑连接（顶点投影参数可能
            // 跨2PI卷绕，不能让MakeEdge自行按顶点参数取界）
            Handle(Geom_Circle) hCirc = new Geom_Circle(circ);
            BRepBuilderAPI_MakeEdge edgeMaker(hCirc, verts[i], verts[i + 1],
                                              alpha1, alpha2);
            if (!edgeMaker.IsDone())
            {
                std::cout << "几何错误：截面圆弧段" << (i + 1) << " 边构造失败" << std::endl;
                return TopoDS_Wire();
            }
            edge = edgeMaker.Edge();
        }
        else
        {
            gp_Pnt p1 = to3D(seg.lineStart);
            gp_Pnt p2 = to3D(seg.lineEnd);
            gp_Lin lin(p1, gp_Dir(gp_Vec(p1, p2)));
            // 直线参数界 [0, 弦长] + 共享顶点
            Handle(Geom_Line) hLin = new Geom_Line(lin);
            BRepBuilderAPI_MakeEdge edgeMaker(hLin, verts[i], verts[i + 1],
                                              0.0, p1.Distance(p2));
            if (!edgeMaker.IsDone())
            {
                std::cout << "几何错误：截面直线段" << (i + 1) << " 边构造失败" << std::endl;
                return TopoDS_Wire();
            }
            edge = edgeMaker.Edge();
        }
        wireMaker.Add(edge);
        if (!wireMaker.IsDone())
        {
            std::cout << "几何错误：截面wire组装失败（段" << (i + 1) << "）" << std::endl;
            return TopoDS_Wire();
        }
    }
    return wireMaker.Wire();
}

// 判断折线是否近似为直线（所有中间点到首末连线的垂距相对偏差极小）
// 用途：离散投影结果中筛掉直线边（已由解析投影覆盖），仅保留曲线边折线
static bool IsPolylineNearlyStraight(const ProjectedPolyline2D& pl)
{
    if (pl.points.size() < 3)
        return true;
    const gp_Pnt2d& a = pl.points.front();
    const gp_Pnt2d& b = pl.points.back();
    double len = a.Distance(b);
    if (len < 1.0e-9)
        return true;
    double ux = (b.X() - a.X()) / len;
    double uy = (b.Y() - a.Y()) / len;
    double maxDev = 0.0;
    for (size_t i = 1; i + 1 < pl.points.size(); ++i)
    {
        double dx = pl.points[i].X() - a.X();
        double dy = pl.points[i].Y() - a.Y();
        double dev = fabs(dx * uy - dy * ux);   // 叉积模 = 垂距 x len
        if (dev > maxDev) maxDev = dev;
    }
    return maxDev < 1.0e-4 * len;
}

// 将轴测投影解析曲线（线段/圆弧/圆）写入 DXF（AC1015 = AutoCAD 2000，兼容 R12+ 读取）
// extraPolylines：解析不支持的曲线边（BSpline 等）离散折线补充，写入 POLYLINE 实体
// 轴测曲线写 DXF。
// spinePolylines（可选）：扫掠脊线的 2D 投影折线，写入独立 "SPINE" 图层：
//   红色(ACI 1 = RGB 255,0,0)、虚线(SPINE_DASHED)、线宽 0.50mm（96dpi 下约 2 像素）；
//   脊线先于模型几何写出（实体顺序靠下），CAD 重生成时模型几何覆盖在脊线之上，
//   且 SPINE 图层可独立冻结/关闭/锁定，保证脊线不会遮挡其他关键几何元素。
static bool WriteAxonometricCurvesToDXF(const std::vector<ProjectedCurve2D>& curves,
                                        const std::vector<ProjectedPolyline2D>& extraPolylines,
                                        const char* file,
                                        const std::vector<ProjectedPolyline2D>* spinePolylines = NULL,
                                        bool writeStraightPolylines = false)
{
    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out(file, DL_Codes::AC1015);
    if (dw == NULL)
    {
        std::cout << "无法创建 DXF 文件: " << file << std::endl;
        return false;
    }

    dxf.writeHeader(*dw);
    dw->sectionEnd();

    dw->sectionTables();
    dxf.writeVPort(*dw);

    // 线型表：CONTINUOUS=实线；SPINE_DASHED=脊线虚线。
    // 脊线杆件长数百（毫米级），标准 DASHED(0.5,-0.25) 会密得像实线，
    // 故定义尺度合适的模式：落笔30、抬笔15（周期45），保证虚线清晰可见。
    dw->tableLinetypes(2);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    {
        double dashPattern[2] = { 30.0, -15.0 };
        dxf.writeLinetype(*dw, DL_LinetypeData("SPINE_DASHED", "__ __ __ __ __",
                                               0, 2, 45.0, dashPattern));
    }
    dw->tableEnd();

    // 图层表：0=模型几何（白/黑随背景）；SPINE=扫掠脊线（红、0.50mm、虚线）
    dw->tableLayers(2);
    dxf.writeLayer(*dw, DL_LayerData("0", 0),
                   DL_Attributes(std::string(""), DL_Codes::white, 100, "CONTINUOUS", 1.0));
    dxf.writeLayer(*dw, DL_LayerData("SPINE", 0),
                   DL_Attributes(std::string(""), DL_Codes::red, 50, "SPINE_DASHED", 1.0));
    dw->tableEnd();

    dw->tableStyle(1);
    dxf.writeStyle(*dw, DL_StyleData("standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "txt", ""));
    dw->tableEnd();

    dxf.writeView(*dw);
    dxf.writeUcs(*dw);

    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    dxf.writeDimStyle(*dw, 1, 1, 1, 1, 1);

    dxf.writeBlockRecord(*dw);
    dw->tableEnd();
    dw->sectionEnd();

    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    // 实体段：全部输出为 LINE 线段。
    // 圆弧/整圆按角度离散拟合成线段（约5度/段，弧最少2段），曲线离散折线也逐段写 LINE，
    // DXF 中不出现 ARC/CIRCLE/POLYLINE 实体，下游只需处理线段。
    dw->sectionEntities();
    int lineCount = 0;
    int fitLineCount = 0;   // 由圆弧/圆/曲线折线拟合产生的线段数
    int spineSegCount = 0;  // 脊线 LINE 数（不计入模型几何 lineCount）

    // 脊线先写：红色虚线、0.50mm 线宽，固定落在 SPINE 图层。
    // 用折线点序列表达，天然兼容直线/曲线/复杂路径（曲线脊线采样为多点即可）。
    if (spinePolylines != NULL)
    {
        DL_Attributes spineAttr("SPINE", DL_Codes::red, 50, "SPINE_DASHED", 1.0);
        for (const auto& sp : *spinePolylines)
        {
            for (size_t i = 1; i < sp.points.size(); ++i)
            {
                const gp_Pnt2d& q1 = sp.points[i - 1];
                const gp_Pnt2d& q2 = sp.points[i];
                if (q1.Distance(q2) < 1.0e-9)
                    continue;
                dxf.writeLine(*dw,
                              DL_LineData(q1.X(), q1.Y(), 0.0, q2.X(), q2.Y(), 0.0),
                              spineAttr);
                spineSegCount++;
            }
        }
    }

    const double fitSegAngleDeg = 5.0;   // 拟合离散密度：度/段
    // 统一写线段入口（退化线段跳过）
    auto writeSegment = [&dxf, &dw, &lineCount](const gp_Pnt2d& p1, const gp_Pnt2d& p2) {
        if (p1.Distance(p2) < 1.0e-9)
            return;
        dxf.writeLine(*dw,
                      DL_LineData(p1.X(), p1.Y(), 0.0, p2.X(), p2.Y(), 0.0),
                      DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
        lineCount++;
    };
    // 圆弧上取点（角度弧度，逆时针，相对圆心）
    auto pointOnCircle = [](const gp_Pnt2d& center, double radius, double angleRad) {
        return gp_Pnt2d(center.X() + radius * cos(angleRad),
                        center.Y() + radius * sin(angleRad));
    };

    for (const auto& c : curves)
    {
        if (c.type == ProjectedCurve2DType::Line)
        {
            writeSegment(gp_Pnt2d(c.startPoint.X(), c.startPoint.Y()),
                         gp_Pnt2d(c.endPoint.X(), c.endPoint.Y()));
        }
        else if (c.type == ProjectedCurve2DType::Circle)
        {
            if (c.radius < 1.0e-9)
                continue;   // 退化圆跳过
            gp_Pnt2d center(c.center.X(), c.center.Y());
            int nbSeg = (int)(360.0 / fitSegAngleDeg);
            gp_Pnt2d prevPt = pointOnCircle(center, c.radius, 0.0);
            for (int i = 1; i <= nbSeg; ++i)
            {
                double a = 2.0 * AXO_PI * i / nbSeg;
                gp_Pnt2d pt = pointOnCircle(center, c.radius, a);
                writeSegment(prevPt, pt);
                prevPt = pt;
                fitLineCount++;
            }
        }
        else // Arc：按角度范围逆时针离散拟合
        {
            if (c.radius < 1.0e-9)
                continue;
            double a1 = c.startAngle;
            double a2 = c.endAngle;
            if (c.isClockwise)
                std::swap(a1, a2);   // 统一为逆时针离散
            while (a2 <= a1 + 1.0e-9)
                a2 += 2.0 * AXO_PI;  // 保证角跨度为正
            double spanDeg = (a2 - a1) * 180.0 / AXO_PI;
            int nbSeg = (int)ceil(spanDeg / fitSegAngleDeg);
            if (nbSeg < 2) nbSeg = 2;
            gp_Pnt2d center(c.center.X(), c.center.Y());
            gp_Pnt2d prevPt = pointOnCircle(center, c.radius, a1);
            for (int i = 1; i <= nbSeg; ++i)
            {
                double a = a1 + (a2 - a1) * i / nbSeg;
                gp_Pnt2d pt = pointOnCircle(center, c.radius, a);
                writeSegment(prevPt, pt);
                prevPt = pt;
                fitLineCount++;
            }
        }
    }

    // 补充：曲线边离散折线（解析不支持的 BSpline/椭圆等），逐段写成 LINE；
    // 直线折线默认跳过（已由解析 LINE 覆盖，避免重复）；
    // writeStraightPolylines=true（PolyHLR 分支）时直线折线也写出。
    int polyCount = 0;
    for (const auto& pl : extraPolylines)
    {
        if (pl.points.size() < 2)
            continue;
        if (!writeStraightPolylines && IsPolylineNearlyStraight(pl))
            continue;
        double totalLen = 0.0;
        for (size_t i = 1; i < pl.points.size(); ++i)
            totalLen += pl.points[i - 1].Distance(pl.points[i]);
        if (totalLen < 0.1)
            continue;   // 碎边跳过
        for (size_t i = 1; i < pl.points.size(); ++i)
        {
            writeSegment(pl.points[i - 1], pl.points[i]);
            fitLineCount++;
        }
        polyCount++;
    }
    dw->sectionEnd();

    dxf.writeObjects(*dw);
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;

    std::cout << "DXF 实体写入完成: 模型LINE=" << lineCount
              << "（解析直线=" << (lineCount - fitLineCount)
              << " 曲线拟合线段=" << fitLineCount
              << "，来自曲线折线" << polyCount << "条）"
              << "，脊线LINE=" << spineSegCount << "(SPINE图层,红色)"
              << " -> " << file << std::endl;
    return true;
}

// U型扫掠轴测图 DXF 测试主函数：
// viewDir/viewUp 指定轴测视角（默认等轴测：视线(1,1,1)，上方向(0,0,1)）
int SweepAxonometricDXFTest(const SweepPathData& path,
                            const SectionProfileData& section,
                            const char* dxfPath,
                            const gp_Dir& viewDir = gp_Dir(1, 1, 1),
                            const gp_Dir& viewUp = gp_Dir(0, 0, 1))
{
    std::cout << "\n--- U型扫掠轴测图 DXF 测试 ---" << std::endl;
    try
    {
        // 1. 参数验证
        if (dxfPath == NULL || dxfPath[0] == '\0')
        {
            std::cout << "参数错误：DXF 输出路径为空" << std::endl;
            return -1;
        }
        if (fabs(viewDir.Dot(viewUp)) > 0.999999)
        {
            std::cout << "参数错误：视线方向与屏幕上方向平行，无法构成轴测视角" << std::endl;
            return -1;
        }
        if (!ValidateSweepPath(path))
        {
            std::cout << "参数错误：扫掠轨迹数据验证失败" << std::endl;
            return -1;
        }
        if (!ValidateSectionProfile(section))
        {
            std::cout << "参数错误：截面轮廓数据验证失败" << std::endl;
            return -1;
        }
        std::cout << "参数验证通过：轨迹" << path.segments.size() << "段，截面" << section.segments.size() << "段" << std::endl;

        // 2. 数据 -> 拓扑 wire
        TopoDS_Wire spineWire = BuildWireFromSweepPath(path);
        if (spineWire.IsNull())
        {
            std::cout << "错误：扫掠轨迹wire构造失败" << std::endl;
            return -1;
        }
        TopoDS_Wire sectionWire = BuildWireFromSection(section);
        if (sectionWire.IsNull())
        {
            std::cout << "错误：截面轮廓wire构造失败" << std::endl;
            return -1;
        }
        std::cout << "轨迹wire与截面wire构造完成" << std::endl;

        // 3. 扫掠建模：生成带面扫掠壳（关键：面参与 HLR 遮挡判定。
        //    若传纯边线框 BuildSweepWireframe 的结果，HLR 无面可判遮挡，
        //    所有边都会被输出为可见边，轴测图没有消隐效果）
        TopoDS_Shape model = BuildSweepShell(spineWire, sectionWire);
        if (model.IsNull())
        {
            std::cout << "错误：扫掠模型生成失败" << std::endl;
            return -1;
        }
        BRepCheck_Analyzer checker(model);
        std::cout << "扫掠模型生成" << (checker.IsValid() ? "，拓扑合法" : "，但拓扑存在问题") << std::endl;

        // 诊断：端盖面生成与有效性检查（两端遮挡依赖端盖面参与 HLR 遮挡判定）
        {
            Bnd_Box modelBox;
            BRepBndLib::Add(model, modelBox);
            double bxm, bym, bzm, bxM, byM, bzM;
            modelBox.Get(bxm, bym, bzm, bxM, byM, bzM);

            int totalFaces = 0, lateralFaces = 0, invalidFaces = 0;
            int capFaces[2] = { 0, 0 };   // [0]=起点端盖(x≈bxm)，[1]=终点端盖(x≈bxM)
            for (TopExp_Explorer fExp(model, TopAbs_FACE); fExp.More(); fExp.Next())
            {
                TopoDS_Face face = TopoDS::Face(fExp.Current());
                totalFaces++;

                BRepCheck_Analyzer faceChecker(face);
                if (!faceChecker.IsValid())
                    invalidFaces++;

                Bnd_Box faceBox;
                BRepBndLib::Add(face, faceBox);
                double fxm, fym, fzm, fxM, fyM, fzM;
                faceBox.Get(fxm, fym, fzm, fxM, fyM, fzM);

                // 端盖候选：平面且 X 方向无厚度（垂直于 +X 轨迹的平面）
                BRepAdaptor_Surface surf(face);
                bool isCap = (surf.GetType() == GeomAbs_Plane &&
                              fabs(fxM - fxm) < 1.0e-3 &&
                              (bxM - bxm) > 1.0e-3);
                if (isCap)
                {
                    int capIdx = (fabs(fxm - bxm) <= fabs(fxm - bxM)) ? 0 : 1;
                    capFaces[capIdx]++;

                    // 端盖外法向（考虑face朝向）：起点端应指向 -X，终点端应指向 +X
                    gp_Dir n = surf.Plane().Axis().Direction();
                    if (face.Orientation() == TopAbs_REVERSED)
                        n.Reverse();
                    double expect = (capIdx == 0) ? -1.0 : 1.0;
                    bool normalOk = (n.X() * expect > 0.0);

                    std::cout << "诊断：端盖(" << (capIdx == 0 ? "起点" : "终点")
                              << ") X=" << fxm
                              << "，Y[" << fym << "," << fyM
                              << "] Z[" << fzm << "," << fzM
                              << "]，外法向X=" << n.X()
                              << (normalOk ? "(朝外,正确)" : "(朝内,警告)")
                              << "，面有效=" << (faceChecker.IsValid() ? "是" : "否")
                              << std::endl;
                }
                else
                {
                    lateralFaces++;
                }
            }
            std::cout << "诊断：模型面总数=" << totalFaces
                      << "（侧面=" << lateralFaces
                      << "，起点端盖=" << capFaces[0]
                      << "，终点端盖=" << capFaces[1] << "）"
                      << "，整体包围盒X[" << bxm << "," << bxM << "]"
                      << "，无效面=" << invalidFaces << std::endl;
        }

        // 诊断：端部遮挡有效性（几何射线测试，独立于 HLR 的直接证据）。
        // 从端盖采样点向观察者方向发射线与模型求交：命中其他面 = 该点被遮挡。
        {
            Bnd_Box bbox;
            BRepBndLib::Add(model, bbox);
            double rxm, rym, rzm, rxM, ryM, rzM;
            bbox.Get(rxm, rym, rzm, rxM, ryM, rzM);
            double xMid = 0.5 * (rxm + rxM);

            // 收集两个端盖面（平面且X方向无厚度），按x排序：[0]=远端 [1]=近端
            TopoDS_Face capF[2];
            double capX[2];
            int nCaps = 0;
            for (TopExp_Explorer fE(model, TopAbs_FACE); fE.More() && nCaps < 2; fE.Next())
            {
                TopoDS_Face f = TopoDS::Face(fE.Current());
                Bnd_Box fb;
                BRepBndLib::Add(f, fb);
                double a1, a2, a3, a4, a5, a6;
                fb.Get(a1, a2, a3, a4, a5, a6);
                if (BRepAdaptor_Surface(f).GetType() == GeomAbs_Plane &&
                    fabs(a4 - a1) < 1.0e-3)
                {
                    capF[nCaps] = f;
                    capX[nCaps] = a1;
                    nCaps++;
                }
            }
            if (nCaps == 2)
            {
                if (capX[0] > capX[1])
                {
                    std::swap(capF[0], capF[1]);
                    std::swap(capX[0], capX[1]);
                }

                // 射线遮挡测试：从 p 沿 d 方向，命中模型任一面（射线参数>容差）即被遮挡
                auto occluded = [&](const gp_Pnt& p, const gp_Dir& d) {
                    gp_Lin ray(p, d);
                    BRepIntCurveSurface_Inter inter;
                    for (inter.Init(model, ray, 1.0e-7); inter.More(); inter.Next())
                        if (inter.W() > 1.0e-4)
                            return true;
                    return false;
                };

                // viewDir 是视线方向（eye->scene），观察者位于 -viewDir 一侧：
                // 射线向 -viewDir 发射；p·viewDir 较小的一端是观察者近端
                gp_Dir toEye(-viewDir.X(), -viewDir.Y(), -viewDir.Z());
                double d0 = capX[0] * viewDir.X();
                double d1 = capX[1] * viewDir.X();
                bool x0NearViewer = (d0 < d1);

                // 轮廓边采样：两端顶点都在同一端的边（端盖边界边），每边取 1/4、1/2、3/4 三点
                int nearTot = 0, nearOcc = 0, farTot = 0, farOcc = 0;
                double nearCapX = 0.0, farCapX = 0.0;
                for (TopExp_Explorer eE(model, TopAbs_EDGE); eE.More(); eE.Next())
                {
                    TopoDS_Edge e = TopoDS::Edge(eE.Current());
                    TopoDS_Vertex v1, v2;
                    TopExp::Vertices(e, v1, v2);
                    if (v1.IsNull() || v2.IsNull())
                        continue;
                    double x1 = BRep_Tool::Pnt(v1).X();
                    double x2 = BRep_Tool::Pnt(v2).X();
                    bool isX0  = (x1 < xMid && x2 < xMid);
                    bool isX1 = (x1 > xMid && x2 > xMid);
                    if (!isX0 && !isX1)
                        continue;
                    // x0 端 = 观察者近端？
                    bool isNearViewer = (isX0 == x0NearViewer);

                    BRepAdaptor_Curve c(e);
                    for (int k = 1; k <= 3; ++k)
                    {
                        gp_Pnt p = c.Value(c.FirstParameter() +
                                           (c.LastParameter() - c.FirstParameter()) * k / 4.0);
                        bool occ = occluded(p, toEye);
                        if (isNearViewer) { nearTot++; if (occ) nearOcc++; }
                        else              { farTot++;  if (occ) farOcc++; }
                    }
                }
                nearCapX = x0NearViewer ? capX[0] : capX[1];
                farCapX  = x0NearViewer ? capX[1] : capX[0];
                std::cout << "诊断：端部遮挡射线测试（采样点向观察者方向）"
                          << " 观察者近端(x=" << nearCapX << ")轮廓被遮挡="
                          << nearOcc << "/" << nearTot
                          << "，观察者远端(x=" << farCapX << ")轮廓被遮挡="
                          << farOcc << "/" << farTot
                          << "（远端部分被遮挡+部分露出=正确消隐）" << std::endl;
            }
        }

        // 4. 轴测投影（解析输出：线段/圆弧/圆，不离散）
        std::vector<TopoDS_Shape> shapes;
        shapes.push_back(model);
        std::vector<ProjectedCurve2D> curves2d;
        if (!ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, curves2d))
        {
            std::cout << "错误：轴测投影计算失败" << std::endl;
            return -1;
        }
        std::cout << "轴测投影完成：解析曲线数=" << curves2d.size()
                  << "（视线(" << viewDir.X() << "," << viewDir.Y() << "," << viewDir.Z()
                  << ")，上方向(" << viewUp.X() << "," << viewUp.Y() << "," << viewUp.Z() << ")）" << std::endl;

        // 5. 补充投影：解析不支持的曲线边（扫掠产生的 BSpline 等）离散为折线；
        //    直线折线在写入时被过滤（已由解析 LINE 覆盖），仅保留曲线部分
        std::vector<ProjectedPolyline2D> polylines;
        ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, polylines);
        std::cout << "离散补充投影完成：折线数=" << polylines.size() << std::endl;

        // 6. 写 DXF
        if (!WriteAxonometricCurvesToDXF(curves2d, polylines, dxfPath))
        {
            std::cout << "错误：轴测图写入 DXF 失败" << std::endl;
            return -1;
        }

        std::cout << "U型扫掠轴测图 DXF 测试通过！" << std::endl;
        return 0;
    }
    catch (const Standard_Failure& e)
    {
        std::cout << "几何计算异常: " << e.GetMessageString() << std::endl;
        return -1;
    }
    catch (const std::exception& e)
    {
        std::cout << "标准异常: " << e.what() << std::endl;
        return -1;
    }
}

// 将截面轮廓数据（2D：直线/圆弧/椭圆）直接写入 DXF 文件（解析实体直出，不做离散拟合）。
// 直线段写 LINE；圆弧写 ARC（度，恒逆时针 angle1->angle2，整圆写 CIRCLE）；
// 椭圆写 ELLIPSE（长轴向量+轴比，参数角弧度）。坐标取截面局部 (U,V)，Z=0。
static bool WriteSectionProfileToDXF(const SectionProfileData& section, const char* file)
{
    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out(file, DL_Codes::AC1015);
    if (dw == NULL)
    {
        std::cout << "无法创建 DXF 文件: " << file << std::endl;
        return false;
    }

    dxf.writeHeader(*dw);
    dw->sectionEnd();

    dw->sectionTables();
    dxf.writeVPort(*dw);

    dw->tableLinetypes(1);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dw->tableEnd();

    dw->tableLayers(1);
    dxf.writeLayer(*dw, DL_LayerData("0", 0),
                   DL_Attributes(std::string(""), DL_Codes::white, 100, "CONTINUOUS", 1.0));
    dw->tableEnd();

    dw->tableStyle(1);
    dxf.writeStyle(*dw, DL_StyleData("standard", 0, 2.5, 1.0, 0.0, 0, 2.5, "txt", ""));
    dw->tableEnd();

    dxf.writeView(*dw);
    dxf.writeUcs(*dw);

    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    dxf.writeDimStyle(*dw, 1, 1, 1, 1, 1);

    dxf.writeBlockRecord(*dw);
    dw->tableEnd();
    dw->sectionEnd();

    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    // 实体段：解析实体直出，不做离散拟合——
    // 直线 -> LINE；圆弧 -> ARC（整圆 -> CIRCLE）；椭圆 -> ELLIPSE
    dw->sectionEntities();
    int lineSegCount = 0;    // LINE 实体数
    int arcCount = 0;        // ARC 实体数
    int circleCount = 0;     // CIRCLE 实体数（整圆弧）
    int ellipseCount = 0;    // ELLIPSE 实体数

    // 弧度 -> DXF 角度（度），规范化到 [0,360)
    auto toDeg = [](double rad) {
        double deg = fmod(rad * 180.0 / AXO_PI, 360.0);
        if (deg < 0.0) deg += 360.0;
        return deg;
    };
    auto writeSeg = [&dxf, &dw](const gp_Pnt2d& p1, const gp_Pnt2d& p2) {
        if (p1.Distance(p2) < 1.0e-9)
            return;
        dxf.writeLine(*dw,
                      DL_LineData(p1.X(), p1.Y(), 0.0, p2.X(), p2.Y(), 0.0),
                      DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
    };

    for (const auto& seg : section.segments)
    {
        if (!seg.isArc)
        {
            // 直线段 -> LINE
            writeSeg(seg.lineStart, seg.lineEnd);
            lineSegCount++;
        }
        else if (seg.isEllipse)
        {
            // 椭圆段 -> ELLIPSE：长轴以"端点相对中心的向量"给出，ratio=短轴/长轴，
            // angle1/angle2 为参数角（弧度）。DXF 椭圆恒按逆时针从 angle1 到 angle2，
            // 顺时针段交换起止参数，输出同一弧段（点集不变）
            double u1 = seg.startAngle;
            double u2 = seg.endAngle;
            if (seg.isClockwise)
                std::swap(u1, u2);
            dxf.writeEllipse(*dw,
                             DL_EllipseData(seg.arcCenter.X(), seg.arcCenter.Y(), 0.0,
                                            seg.majorRadius * cos(seg.majorAxisAngle),
                                            seg.majorRadius * sin(seg.majorAxisAngle),
                                            0.0,
                                            seg.minorRatio, u1, u2),
                             DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
            ellipseCount++;
        }
        else
        {
            // 圆弧段 -> ARC（角度单位为度，恒按逆时针从 angle1 到 angle2）
            if (seg.arcRadius < 1.0e-9)
                continue;
            double span = seg.isClockwise ? (seg.startAngle - seg.endAngle)
                                          : (seg.endAngle - seg.startAngle);
            if (span >= 2.0 * AXO_PI - 1.0e-9)
            {
                // 整圆：DXF ARC 无法表达（angle1==angle2 为零弧），写 CIRCLE
                dxf.writeCircle(*dw,
                                DL_CircleData(seg.arcCenter.X(), seg.arcCenter.Y(), 0.0,
                                              seg.arcRadius),
                                DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
                circleCount++;
            }
            else
            {
                // 顺时针弧交换起止角（DXF 从 angle1 逆时针画到 angle2，点集不变）
                double a1 = seg.isClockwise ? seg.endAngle : seg.startAngle;
                double a2 = seg.isClockwise ? seg.startAngle : seg.endAngle;
                dxf.writeArc(*dw,
                             DL_ArcData(seg.arcCenter.X(), seg.arcCenter.Y(), 0.0,
                                        seg.arcRadius, toDeg(a1), toDeg(a2)),
                             DL_Attributes("0", DL_Codes::white, -1, "BYLAYER", 1.0));
                arcCount++;
            }
        }
    }
    dw->sectionEnd();

    dxf.writeObjects(*dw);
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;

    std::cout << "截面轮廓 DXF 写入完成: LINE=" << lineSegCount
              << " ARC=" << arcCount
              << " CIRCLE=" << circleCount
              << " ELLIPSE=" << ellipseCount
              << " -> " << file << std::endl;
    return true;
}

// 从原始 txt 数据文件读取截面轮廓，按文件中记录出现顺序生成轮廓段。支持三类记录：
//   Segment[start:x,y,z;end:x,y,z;]                                            -> 直线段
//   Arc[Center:x,y,z;Radius:r;PlaneNormalDir:nx,ny,nz;;start:x,y,z;end:x,y,z;] -> 圆弧
//   PolyLn:[Pt0:x,y,z;Pt1:x,y,z;...]                                           -> 相邻点依次生成直线段（折线闭合边）
// 圆弧方向由 PlaneNormalDir 的 Z 分量给出：<0 顺时针（从+Z看），>0 逆时针；
// 半径取圆心-起点距离重算，并与文件 Radius 字段交叉校验（容差 1e-3），保证弧严格过端点。
static bool LoadSectionProfileFromTxt(const char* file, SectionProfileData& profile)
{
    std::ifstream fin(file);
    if (!fin.is_open())
    {
        std::cout << "无法打开截面数据文件: " << file << std::endl;
        return false;
    }

    // 数值捕获组（含符号/小数/科学计数法）；外层括号为捕获组，每次插入 N 产生一个分组
    const std::string N = "(" + std::string(R"([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)") + ")";
    const std::string segPat =
        "Segment\\s*\\[\\s*start\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N +
        "\\s*;\\s*end\\s*:\\s*"   + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N;
    const std::string arcPat =
        "Arc\\s*\\[\\s*Center\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N +
        "\\s*;\\s*Radius\\s*:\\s*" + N +
        "\\s*;\\s*PlaneNormalDir\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N +
        "\\s*;\\s*;\\s*start\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N +
        "\\s*;\\s*end\\s*:\\s*"   + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N;
    const std::string polyPat = R"(PolyLn\s*:\s*\[([^\]]*)\])";
    const std::string ptPat =
        "Pt\\s*[0-9]+\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N;

    std::regex segRe(segPat, std::regex::icase);
    std::regex arcRe(arcPat, std::regex::icase);
    std::regex polyRe(polyPat, std::regex::icase);
    std::regex ptRe(ptPat, std::regex::icase);

    auto wrap2Pi = [](double a) {
        while (a < 0.0)           a += 2.0 * AXO_PI;
        while (a >= 2.0 * AXO_PI) a -= 2.0 * AXO_PI;
        return a;
    };
    auto addLine = [&](const gp_Pnt2d& p1, const gp_Pnt2d& p2) {
        SectionSeg s;
        s.isArc = false;
        s.lineStart = p1;
        s.lineEnd   = p2;
        s.lineLength = p1.Distance(p2);
        profile.segments.push_back(s);
    };

    int lineCount = 0, arcCount = 0, cwCount = 0, ccwCount = 0, warnCount = 0;
    std::string line;
    while (std::getline(fin, line))
    {
        // 去除可能的 UTF-8 BOM（首行）
        if (line.size() >= 3 &&
            (unsigned char)line[0] == 0xEF &&
            (unsigned char)line[1] == 0xBB &&
            (unsigned char)line[2] == 0xBF)
            line.erase(0, 3);

        std::smatch m;
        if (std::regex_search(line, m, segRe))
        {
            // 捕获组 1-3=start，4-6=end（z 分量不取，轮廓为 2D）
            addLine(gp_Pnt2d(std::stod(m[1].str()), std::stod(m[2].str())),
                    gp_Pnt2d(std::stod(m[4].str()), std::stod(m[5].str())));
            lineCount++;
        }
        else if (std::regex_search(line, m, arcRe))
        {
            // 1-3=Center，4=Radius，5-7=PlaneNormalDir，8-10=start，11-13=end
            double cx = std::stod(m[1].str());
            double cy = std::stod(m[2].str());
            double rFile = std::stod(m[4].str());
            double nz = std::stod(m[7].str());
            gp_Pnt2d p1(std::stod(m[8].str()),  std::stod(m[9].str()));
            gp_Pnt2d p2(std::stod(m[11].str()), std::stod(m[12].str()));

            SectionSeg s;
            s.isArc = true;
            s.arcCenter = gp_Pnt2d(cx, cy);
            s.lineStart = p1;
            s.lineEnd   = p2;
            s.lineLength = 0.0;

            // 半径由圆心-端点距离重算，保证弧严格过端点；与文件 Radius 字段交叉校验
            s.arcRadius = p1.Distance(s.arcCenter);
            double rEnd = p2.Distance(s.arcCenter);
            if (rFile > 0.0 && fabs(rFile - s.arcRadius) > 1.0e-3)
            {
                std::cout << "警告：圆弧Radius字段与端点重算值不符，圆心("
                          << cx << "," << cy << ") R文件=" << rFile
                          << " R重算=" << s.arcRadius << std::endl;
                warnCount++;
            }
            if (fabs(rEnd - s.arcRadius) > 1.0e-3)
            {
                std::cout << "警告：圆弧端点到圆心距离不一致，圆心("
                          << cx << "," << cy << ") rStart=" << s.arcRadius
                          << " rEnd=" << rEnd << std::endl;
                warnCount++;
            }

            // 端点相对圆心的真实数学角，按法向方向写入标准数学角区间：
            // 逆时针 endAngle>startAngle；顺时针 startAngle>endAngle
            double a1 = atan2(p1.Y() - cy, p1.X() - cx);
            double a2 = atan2(p2.Y() - cy, p2.X() - cx);
            s.isClockwise = (nz < 0.0);
            s.startAngle = a1;
            s.endAngle = s.isClockwise ? a1 - wrap2Pi(a1 - a2)   // 顺时针：角度递减
                                       : a1 + wrap2Pi(a2 - a1);  // 逆时针：角度递增
            profile.segments.push_back(s);
            arcCount++;
            if (s.isClockwise) cwCount++; else ccwCount++;
        }
        else if (std::regex_search(line, m, polyRe))
        {
            // PolyLn：提取点序列，相邻点依次生成直线段
            std::vector<gp_Pnt2d> pts;
            const std::string inner = m[1].str();
            for (std::sregex_iterator it(inner.begin(), inner.end(), ptRe), endIt; it != endIt; ++it)
                pts.emplace_back(std::stod((*it)[1].str()), std::stod((*it)[2].str()));
            for (size_t i = 1; i < pts.size(); ++i)
            {
                addLine(pts[i - 1], pts[i]);
                lineCount++;
            }
        }
        // 其余行（标题、空行等）忽略
    }

    if (profile.segments.empty())
    {
        std::cout << "错误：未从 " << file << " 解析到任何轮廓段" << std::endl;
        return false;
    }

    // 闭合性检查：末段终点与首段起点应重合
    double closeGap = profile.segments.front().lineStart.Distance(profile.segments.back().lineEnd);
    if (closeGap > 1.0e-3)
    {
        std::cout << "警告：轮廓未闭合，首起点与末终点距离=" << closeGap << std::endl;
        warnCount++;
    }

    std::cout << "截面数据读取完成: " << file
              << "，共 " << profile.segments.size() << " 段（直线=" << lineCount
              << " 圆弧=" << arcCount << "[" << cwCount << "顺时针+" << ccwCount
              << "逆时针]），闭合间隙=" << closeGap;
    if (warnCount > 0) std::cout << "，警告 " << warnCount << " 条";
    std::cout << std::endl;
    return true;
}

// L型扫掠 DXF 测试：
// 轨迹：L 型脊线，两段直线 (0,0,0)->(200,0,0)->(200,200,0)
// 截面：从 test.txt 读取真实轮廓（Segment/Arc 记录 + PolyLn 折线闭合边），
// 数据(x,y)作为局部UV，截面平面法向 +X（垂直第一段脊线方向），U轴 +Y
int LineSweepDXFTest()
{
    std::cout << "\n--- 闭合矩形脊线扫掠 DXF 测试（多尺寸验证）---" << std::endl;

    // 从原始 txt 文件读取截面轮廓（替代硬编码数据）
    SectionProfileData profile;
    profile.planeOrigin = gp_Pnt(0, 0, 0);
    profile.planeNormal = gp_Dir(1, 0, 0);
    profile.planeXRef   = gp_Dir(0, 1, 0);
    if (!LoadSectionProfileFromTxt("test.txt", profile))
        return -1;

    // 截面 2D 坐标 (U,V) 经 to3D 映射为 3D：
    //   3D = planeOrigin + U*planeXRef + V*(planeNormal x planeXRef)
    // 当前设置下 V 方向 = +Z，txt 中 V 值约 1277~1324 → 扫掠体 Z 偏高。
    // 计算 V 最小值并平移 planeOrigin.Z，使截面最低点 Z=0（与脊线同高）。
    {
        double minV = 1.0e30;
        for (const auto& seg : profile.segments)
        {
            if (seg.lineStart.Y() < minV) minV = seg.lineStart.Y();
            if (seg.lineEnd.Y()   < minV) minV = seg.lineEnd.Y();
            if (seg.isArc && seg.arcCenter.Y() - seg.arcRadius < minV)
                minV = seg.arcCenter.Y() - seg.arcRadius;
        }
        if (minV < 1.0e29)
            profile.planeOrigin.SetZ(-minV);
    }

    // 将输入的截面轮廓数据（2D 直线/圆弧/椭圆）写入 DXF（解析实体直出）
    WriteSectionProfileToDXF(profile, "section_profile.dxf");

    // 闭合矩形脊线生成 lambda：宽 w、高 h，四段直线闭合矩形
    auto makeRectPath = [](double w, double h) {
        SweepPathData path;
        SweepPathSeg s1; s1.isArc = false;
        s1.start = gp_Pnt(0, 0, 0);   s1.end = gp_Pnt(w, 0, 0);
        SweepPathSeg s2; s2.isArc = false;
        s2.start = gp_Pnt(w, 0, 0);   s2.end = gp_Pnt(w, -h, 0);
        SweepPathSeg s3; s3.isArc = false;
        s3.start = gp_Pnt(w, -h, 0);  s3.end = gp_Pnt(0, -h, 0);
        SweepPathSeg s4; s4.isArc = false;
        s4.start = gp_Pnt(0, -h, 0);  s4.end = gp_Pnt(0, 0, 0);
        path.segments = {s1, s2, s3, s4};
        return path;
    };

    // 测试 1：大尺寸 2000x500
    {
        std::cout << "\n[测试1] 闭合矩形 2000x500" << std::endl;
        auto path = makeRectPath(2000, 500);
        int r = SweepAxonometricDXFTest(path, profile, "sweep_rect_2000x500.dxf",
                                        gp_Dir(1, 1, 1), gp_Dir(0, 0, 1));
        if (r != 0) { std::cout << "[测试1] 失败！" << std::endl; return r; }
    }

    // 测试 2：中尺寸 1000x300
    {
        std::cout << "\n[测试2] 闭合矩形 1000x300" << std::endl;
        auto path = makeRectPath(1000, 300);
        int r = SweepAxonometricDXFTest(path, profile, "sweep_rect_1000x300.dxf",
                                        gp_Dir(1, 1, 1), gp_Dir(0, 0, 1));
        if (r != 0) { std::cout << "[测试2] 失败！" << std::endl; return r; }
    }

    // 测试 3：小尺寸 500x200
    {
        std::cout << "\n[测试3] 闭合矩形 500x200" << std::endl;
        auto path = makeRectPath(500, 200);
        int r = SweepAxonometricDXFTest(path, profile, "sweep_rect_500x200.dxf",
                                        gp_Dir(1, 1, 1), gp_Dir(0, 0, 1));
        if (r != 0) { std::cout << "[测试3] 失败！" << std::endl; return r; }
    }

    std::cout << "\n========== 所有闭合矩形尺寸测试通过！==========" << std::endl;
    return 0;
}

// 从 txt 数据文件读取扫掠轨迹段（按文件中记录出现顺序，加载方式与
// LoadSectionProfileFromTxt 一致），当前支持：
//   Segment[start:x,y,z;end:x,y,z;] -> 直线段（含 Z 分量）
// 文件允许含标题行/空行；首行 UTF-8 BOM 自动去除。
static bool LoadSweepPathFromTxt(const char* file, SweepPathData& path)
{
    std::ifstream fin(file);
    if (!fin.is_open())
    {
        std::cout << "无法打开轨迹数据文件: " << file << std::endl;
        return false;
    }

    // 数值捕获组（含符号/小数/科学计数法）；每次插入 N 产生一个分组
    const std::string N = "(" + std::string(R"([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)") + ")";
    const std::string segPat =
        "Segment\\s*\\[\\s*start\\s*:\\s*" + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N +
        "\\s*;\\s*end\\s*:\\s*"   + N + "\\s*,\\s*" + N + "\\s*,\\s*" + N;
    std::regex segRe(segPat, std::regex::icase);

    int lineCount = 0;
    std::string line;
    while (std::getline(fin, line))
    {
        // 去除可能的 UTF-8 BOM（首行）
        if (line.size() >= 3 &&
            (unsigned char)line[0] == 0xEF &&
            (unsigned char)line[1] == 0xBB &&
            (unsigned char)line[2] == 0xBF)
            line.erase(0, 3);

        std::smatch m;
        if (std::regex_search(line, m, segRe))
        {
            SweepPathSeg seg;
            seg.isArc = false;
            // 捕获组 1-3=start，4-6=end（轨迹保留 Z 分量）
            seg.start = gp_Pnt(std::stod(m[1].str()), std::stod(m[2].str()), std::stod(m[3].str()));
            seg.end   = gp_Pnt(std::stod(m[4].str()), std::stod(m[5].str()), std::stod(m[6].str()));
            path.segments.push_back(seg);
            lineCount++;
        }
        // 其余行（标题、空行等）忽略
    }

    if (path.segments.empty())
    {
        std::cout << "错误：未从 " << file << " 解析到任何轨迹段" << std::endl;
        return false;
    }

    std::cout << "轨迹数据读取完成: " << file << "，共 " << lineCount << " 个直线段" << std::endl;
    return true;
}

// 计算轨迹段在起点处的切向：
//   直线段 = start->end 方向；圆弧段 = arcAxis × (arcCenter->start)
// （约定 start->end 沿 arcAxis 逆时针，故起点切向为轴叉径向）
static gp_Dir GetSweepPathSegStartTangent(const SweepPathSeg& seg)
{
    gp_Vec tangent;
    if (seg.isArc)
    {
        gp_Vec radial(seg.arcCenter, seg.start);
        tangent = gp_Vec(seg.arcAxis).Crossed(radial);
    }
    else
    {
        tangent = gp_Vec(seg.start, seg.end);
    }
    if (tangent.Magnitude() < 1.0e-9)
        throw Standard_ConstructionError("轨迹段退化，无法计算起点切向");
    return gp_Dir(tangent);
}

// 轨迹文件加载扫掠 DXF 测试：
// 轨迹从 testPath.txt 加载（Segment 记录，允许乱序），由
// OrderSweepPathSegments 重排为首尾相连的有序链（缺少顶边的 U 型开口轮廓）；
// 截面框架由重排后首段动态确定，截面轮廓从 test.txt 加载。
int UnorderedPathSweepDXFTest()
{
    std::cout << "\n--- 轨迹文件加载扫掠 DXF 测试（U型开口轮廓）---" << std::endl;

    // 1. 从 testPath.txt 加载原始轨迹段（按文件记录顺序）
    SweepPathData rawPath;
    if (!LoadSweepPathFromTxt("testPath.txt", rawPath))
        return -1;

    // 2. 重排（轮廓顺序正确 + 不丢边）
    SweepPathData path;
    if (!OrderSweepPathSegments(rawPath, path))
        return -1;

    // 3. 截面框架由重排后首段动态确定：
    //    Normal=首段起点切向（截面垂直于轨迹）；
    //    XRef=+Z×Normal，使 V=Normal×XRef=+Z（截面 V 方向恒为 +Z）；
    //    Origin=首段起点+MoveDir（见下方横向偏移特殊处理）。
    const SweepPathSeg& firstSeg = path.segments.front();
    SectionProfileData profile;
    profile.planeNormal = GetSweepPathSegStartTangent(firstSeg);
    const gp_Vec zVec(0, 0, 1);
    gp_Vec xrefVec = zVec.Crossed(gp_Vec(profile.planeNormal));
    if (xrefVec.Magnitude() < 1.0e-9)
    {
        std::cout << "错误：首段起点切向接近±Z，无法保持截面 V=+Z" << std::endl;
        return -1;
    }
    profile.planeXRef = gp_Dir(xrefVec);

    // 特殊处理：截面横向偏移。MoveDir = (T × +Z) * Dis，其中
    //   T=脊线方向（首段起点切向），T × +Z 即"脊线方向与 +Z 构成平面"的
    //   法线方向（位于水平面内、垂直于脊线，与 XRef=+Z×T 反向）；
    //   Dis 为固定偏移量，默认 30，允许为负数（取负即反向偏移）。
    const double moveDis = 30.0;
    gp_Vec moveDir = gp_Vec(profile.planeNormal).Crossed(zVec) * moveDis;
    profile.planeOrigin = firstSeg.start.Translated(moveDir);
    std::cout << "截面横向偏移：Dis=" << moveDis
              << "，偏移方向(" << moveDir.X() / moveDis << ","
              << moveDir.Y() / moveDis << "," << moveDir.Z() / moveDis
              << ")，偏移后起点(" << profile.planeOrigin.X() << ","
              << profile.planeOrigin.Y() << "," << profile.planeOrigin.Z() << ")"
              << std::endl;

    if (!LoadSectionProfileFromTxt("test2.txt", profile))
        return -1;

    // 平移 planeOrigin.Z，使截面最低点 Z=首段起点 Z（与脊线同高，锚点落在脊线起点）
    {
        double minV = 1.0e30;
        for (const auto& seg : profile.segments)
        {
            if (seg.lineStart.Y() < minV) minV = seg.lineStart.Y();
            if (seg.lineEnd.Y()   < minV) minV = seg.lineEnd.Y();
            if (seg.isArc && seg.arcCenter.Y() - seg.arcRadius < minV)
                minV = seg.arcCenter.Y() - seg.arcRadius;
        }
        if (minV < 1.0e29)
            profile.planeOrigin.SetZ(firstSeg.start.Z() - minV);
    }

    // 4. 扫掠出图
    int r = SweepAxonometricDXFTest(path, profile, "sweep_unordered_u_path.dxf",
                                    gp_Dir(1, -2, 1), gp_Dir(0, 0, 1));
    if (r != 0)
    {
        std::cout << "轨迹文件加载扫掠 DXF 测试失败！" << std::endl;
        return r;
    }

    std::cout << "========== 轨迹文件加载扫掠 DXF 测试通过！==========" << std::endl;
    return 0;
}


// =====================================================================
// JSON 线框扫掠
// =====================================================================
// 数据格式（顶层为数组，每个元素定义一次"闭合多段线截面沿单段路径扫掠"）：
//   [ { "Section":[ {"PlaneNormal":{"x":..,"y":..,"z":..}},
//                   {"Type":"PolyLn","Pt0":{..},...,"PtN":{..}} ],
//       "Sweep":  [ {"PlaneNormal":{..}},
//                   {"Type":"Segment","Start":{..},"End":{..}} ] }, ... ]
// 注意：JSON 对象键无序，Pt 键可能按字典序排列（Pt10 会排在 Pt2 之前），
//       必须按下标数值排序后再顺序连线，否则轮廓顺序错误。

// JSON 单条扫掠定义
struct JsonSweepItem
{
    std::vector<gp_Pnt2d> sectionPts; // 截面多段线顶点（截面局部 UV 坐标，首末点重合=闭合）
    gp_Dir sectionNormal;             // 截面定义平面法向（截面局部 +Z，记录备查）
    gp_Pnt sweepStart;                // 扫掠路径起点（兼作截面放置锚点）
    gp_Pnt sweepEnd;                  // 扫掠路径终点
    gp_Dir sweepPlaneNormal;          // 扫掠平面法向（决定截面姿态，见 BuildSolidFromJsonItem）
    double startAng;                  // 起点切角角度（度），0=垂直端面不处理
    double endAng;                    // 终点切角角度（度），0=垂直端面不处理
};

// 解析 {"x":..,"y":..,"z":..} 为 gp_Pnt
static bool ParseJsonPoint(const nlohmann::json& j, gp_Pnt& out, const std::string& what)
{
    if (!j.is_object() || !j.contains("x") || !j.contains("y") || !j.contains("z"))
    {
        std::cout << "JSON错误：" << what << " 缺少 x/y/z 分量" << std::endl;
        return false;
    }
    out.SetCoord(j["x"].get<double>(), j["y"].get<double>(), j["z"].get<double>());
    return true;
}

// 大小写不敏感的 JSON 点解析：同时接受 "x"/"X"、"y"/"Y"、"z"/"Z" 三种键名。
// 用于解析 Box 条目（既有数据使用大写键 "X/Y/Z"），与既有小写解析器互不影响。
static bool ParseJsonPointCI(const nlohmann::json& j, gp_Pnt& out, const std::string& what)
{
    if (!j.is_object())
    {
        std::cout << "JSON错误：" << what << " 不是对象" << std::endl;
        return false;
    }
    bool hasX = j.contains("x") || j.contains("X");
    bool hasY = j.contains("y") || j.contains("Y");
    bool hasZ = j.contains("z") || j.contains("Z");
    if (!hasX || !hasY || !hasZ)
    {
        std::cout << "JSON错误：" << what << " 缺少 x/y/z（或 X/Y/Z）分量" << std::endl;
        return false;
    }
    double xv = j.contains("x") ? j["x"].get<double>() : j["X"].get<double>();
    double yv = j.contains("y") ? j["y"].get<double>() : j["Y"].get<double>();
    double zv = j.contains("z") ? j["z"].get<double>() : j["Z"].get<double>();
    out.SetCoord(xv, yv, zv);
    return true;
}

// 从 txt 文件加载 JSON 线框扫掠定义。
// 错误处理：文件不存在/读取失败/JSON 语法错误/结构或字段非法，均输出原因并返回 false。
static bool LoadJsonSweepItems(const char* file, std::vector<JsonSweepItem>& items)
{
    items.clear();

    // 1. 读取文件全部内容（二进制方式，随后手动去 BOM）
    std::ifstream fin(file, std::ios::binary);
    if (!fin.is_open())
    {
        std::cout << "错误：无法打开JSON数据文件: " << file << std::endl;
        return false;
    }
    std::stringstream ss;
    ss << fin.rdbuf();
    if (fin.bad())
    {
        std::cout << "错误：JSON数据文件读取失败: " << file << std::endl;
        return false;
    }
    std::string text = ss.str();
    if (text.size() >= 3 &&
        (unsigned char)text[0] == 0xEF &&
        (unsigned char)text[1] == 0xBB &&
        (unsigned char)text[2] == 0xBF)
        text.erase(0, 3);

    // 2. JSON 解析（nlohmann/json）
    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const nlohmann::json::parse_error& e)
    {
        std::cout << "错误：JSON格式错误: " << e.what() << std::endl;
        return false;
    }
    if (!root.is_array())
    {
        std::cout << "错误：JSON顶层必须是数组" << std::endl;
        return false;
    }

    // 3. 逐条解析扫掠定义
    for (size_t idx = 0; idx < root.size(); ++idx)
    {
        const nlohmann::json& elem = root[idx];
        JsonSweepItem item;
        try
        {
            if (!elem.contains("Section") || !elem["Section"].is_array() ||
                !elem.contains("Sweep")   || !elem["Sweep"].is_array())
                throw std::runtime_error("缺少 Section/Sweep 数组");

            // --- Section：PlaneNormal + PolyLn 顶点 ---
            bool gotSecNormal = false, gotPoly = false;
            for (const auto& sub : elem["Section"])
            {
                if (sub.contains("PlaneNormal"))
                {
                    gp_Pnt n;
                    if (!ParseJsonPoint(sub["PlaneNormal"], n, "Section.PlaneNormal"))
                        throw std::runtime_error("Section.PlaneNormal 非法");
                    gp_Vec nv(n.X(), n.Y(), n.Z());
                    if (nv.Magnitude() < 1.0e-9)
                        throw std::runtime_error("Section.PlaneNormal 为零向量");
                    item.sectionNormal = gp_Dir(nv);
                    gotSecNormal = true;
                }
                else if (sub.contains("Type") && sub["Type"].get<std::string>() == "PolyLn")
                {
                    // 收集 Pt<下标> 键并按下标数值排序（字典序会使 Pt10 排在 Pt2 前）
                    std::vector<std::pair<int, gp_Pnt2d>> pts;
                    for (auto it = sub.begin(); it != sub.end(); ++it)
                    {
                        const std::string& key = it.key();
                        if (key.size() > 2 && key[0] == 'P' && key[1] == 't' &&
                            std::isdigit((unsigned char)key[2]))
                        {
                            int pi = std::stoi(key.substr(2));
                            gp_Pnt p;
                            if (!ParseJsonPoint(it.value(), p, key))
                                throw std::runtime_error(key + " 非法");
                            // 截面在自身平面内定义，取 x/y 作为局部 UV（z 为面内坐标，应≈0）
                            pts.push_back(std::make_pair(pi, gp_Pnt2d(p.X(), p.Y())));
                        }
                    }
                    std::sort(pts.begin(), pts.end(),
                              [](const std::pair<int, gp_Pnt2d>& a,
                                 const std::pair<int, gp_Pnt2d>& b) { return a.first < b.first; });
                    for (const auto& kv : pts)
                        item.sectionPts.push_back(kv.second);
                    gotPoly = !item.sectionPts.empty();
                }
            }
            if (!gotSecNormal) throw std::runtime_error("Section 缺少 PlaneNormal");
            if (!gotPoly)      throw std::runtime_error("Section 缺少 PolyLn 顶点");

            // --- Sweep：PlaneNormal + Segment(Start/End) + 可选角度对象{StartAng,EndAng} ---
            bool gotSwpNormal = false, gotSeg = false;
            item.startAng = 0.0;
            item.endAng = 0.0;
            for (const auto& sub : elem["Sweep"])
            {
                if (sub.contains("PlaneNormal"))
                {
                    gp_Pnt n;
                    if (!ParseJsonPoint(sub["PlaneNormal"], n, "Sweep.PlaneNormal"))
                        throw std::runtime_error("Sweep.PlaneNormal 非法");
                    gp_Vec nv(n.X(), n.Y(), n.Z());
                    if (nv.Magnitude() < 1.0e-9)
                        throw std::runtime_error("Sweep.PlaneNormal 为零向量");
                    item.sweepPlaneNormal = gp_Dir(nv);
                    gotSwpNormal = true;
                }
                else if (sub.contains("Type") && sub["Type"].get<std::string>() == "Segment")
                {
                    if (!sub.contains("Start") || !sub.contains("End"))
                        throw std::runtime_error("Segment 缺少 Start/End");
                    if (!ParseJsonPoint(sub["Start"], item.sweepStart, "Sweep.Start") ||
                        !ParseJsonPoint(sub["End"],   item.sweepEnd,   "Sweep.End"))
                        throw std::runtime_error("Segment 端点非法");
                    gotSeg = true;
                }
                else if (sub.contains("StartAng") || sub.contains("EndAng"))
                {
                    // 切角角度（度）：0=不切；字段缺失按 0 处理
                    if (sub.contains("StartAng")) item.startAng = sub["StartAng"].get<double>();
                    if (sub.contains("EndAng"))   item.endAng   = sub["EndAng"].get<double>();
                    if (fabs(item.startAng) >= 90.0 || fabs(item.endAng) >= 90.0)
                        throw std::runtime_error("切角角度必须在(-90,90)范围内");
                }
            }
            if (!gotSwpNormal) throw std::runtime_error("Sweep 缺少 PlaneNormal");
            if (!gotSeg)       throw std::runtime_error("Sweep 缺少 Segment");

            // --- 数据校验 ---
            if (item.sectionPts.size() < 3)
                throw std::runtime_error("截面多段线顶点不足(至少需要3个)");
            if (item.sectionPts.front().Distance(item.sectionPts.back()) > 1.0e-6)
                throw std::runtime_error("截面多段线不闭合（首末点距离>1e-6）");
            if (item.sweepStart.Distance(item.sweepEnd) < 1.0e-9)
                throw std::runtime_error("扫掠路径退化（起点终点重合）");
        }
        catch (const std::exception& e)
        {
            std::cout << "JSON错误：第" << (idx + 1) << " 条扫掠定义无效：" << e.what() << std::endl;
            return false;
        }
        items.push_back(item);
    }

    if (items.empty())
    {
        std::cout << "错误：JSON中没有任何扫掠定义" << std::endl;
        return false;
    }
    std::cout << "JSON数据读取完成: " << file << "，共 " << items.size() << " 条扫掠定义" << std::endl;
    return true;
}

// 切角方式开关：false = 梯形切（切面沿 uMax 侧"上边框"向内旋转，uMax 棱保持全长）；
//               true  = 倒梯形切（切面沿 uMin 侧"下边框"向内旋转，uMin 棱保持全长）。
// 两种切法切面均向内旋转，切除量恒为正，几何严格限制在原始扫掠体内部，不外伸。
static const bool g_invertedTrapezoidCut = true;

// 单条 JSON 扫掠定义 -> 带端盖封闭扫掠壳（复用既有扫掠管线）
// 截面姿态约定：
//   截面平面法向 = 路径切向 T（截面垂直于路径）；
//   截面局部 U 轴 = n_sweep × T（截面镜像到扫掠平面另一侧），
//   从而 V = T × U，即截面局部 Y 恒指向扫掠平面法向一侧——同一框架平面的
//   所有杆件（无论水平段还是竖直段）截面深度方向保持一致。
static TopoDS_Shape BuildSolidFromJsonItem(const JsonSweepItem& item, size_t index)
{
    const std::string tag = "第" + std::to_string(index) + "条扫掠定义：";

    // 1. 扫掠路径（单段直线）
    SweepPathData rawPath;
    SweepPathSeg seg;
    seg.isArc = false;
    seg.start = item.sweepStart;
    seg.end   = item.sweepEnd;
    rawPath.segments.push_back(seg);
    if (!ValidateSweepPath(rawPath))
    {
        std::cout << tag << "路径验证失败" << std::endl;
        return TopoDS_Shape();
    }

    SweepPathData path;
    if (!OrderSweepPathSegments(rawPath, path))
        return TopoDS_Shape();

    // 2. 截面框架
    gp_Dir tangent(gp_Vec(item.sweepStart, item.sweepEnd));
    gp_Vec uVec = gp_Vec(item.sweepPlaneNormal).Crossed(gp_Vec(tangent));
    if (uVec.Magnitude() < 1.0e-9)
    {
        std::cout << tag << "路径切向与扫掠平面法向平行，无法确定截面姿态" << std::endl;
        return TopoDS_Shape();
    }
    SectionProfileData profile;
    profile.planeOrigin = item.sweepStart;
    profile.planeNormal = tangent;
    profile.planeXRef   = gp_Dir(uVec);

    // 3. 截面轮廓：多段线顶点 -> 直线段
    for (size_t i = 0; i + 1 < item.sectionPts.size(); ++i)
    {
        SectionSeg s;
        s.isArc = false;
        s.lineStart  = item.sectionPts[i];
        s.lineEnd    = item.sectionPts[i + 1];
        s.lineLength = s.lineStart.Distance(s.lineEnd);
        profile.segments.push_back(s);
    }
    if (!ValidateSectionProfile(profile))
    {
        std::cout << tag << "截面轮廓验证失败" << std::endl;
        return TopoDS_Shape();
    }

    // ---- 无切角：原始路径 -> wire -> 封闭扫掠壳 ----
    if (fabs(item.startAng) < 1.0e-9 && fabs(item.endAng) < 1.0e-9)
    {
        TopoDS_Wire spineWire   = BuildWireFromSweepPath(path);
        TopoDS_Wire sectionWire = BuildWireFromSection(profile);
        if (spineWire.IsNull() || sectionWire.IsNull())
        {
            std::cout << tag << "wire 构造失败" << std::endl;
            return TopoDS_Shape();
        }
        TopoDS_Shape shell = BuildSweepShell(spineWire, sectionWire);
        if (shell.IsNull())
        {
            std::cout << tag << "扫掠壳生成失败" << std::endl;
            return TopoDS_Shape();
        }
        return shell;
    }

    // ---- 有切角：直接扫掠 -> 实体化 -> 倾斜半空间布尔求交 ----
    // 角度语义：斜切面沿扫掠体一条边框向内旋转，旋转轴 = 扫掠平面法向 n_sweep
    //   （斜切发生在扫掠平面内），枢轴点 = 路径端点 + pivotU·U。
    //   梯形切(g_invertedTrapezoidCut=false)：pivotU=uMax，切面沿上边框向内，
    //     uMax 棱保持全长 L，其余部分端部切短 (uMax-u)·tanθ；
    //   倒梯形切(g_invertedTrapezoidCut=true)：pivotU=uMin，切面沿下边框向内，
    //     uMin 棱保持全长 L，其余部分端部切短 (u-uMin)·tanθ。
    //   两种切法切除量恒为正，几何严格收在原始扫掠体范围内，不外伸、不相交。
    double uMin = 1.0e30, uMax = -1.0e30;
    for (const auto& p : item.sectionPts)
    {
        if (p.X() < uMin) uMin = p.X();
        if (p.X() > uMax) uMax = p.X();
    }
    // 开关 -> 枢轴偏移与两端旋转方向（梯形/倒梯形恰好相反）
    double pivotU = g_invertedTrapezoidCut ? uMin : uMax;
    double endRotSign   = g_invertedTrapezoidCut ? +1.0 : -1.0;
    double startRotSign = g_invertedTrapezoidCut ? -1.0 : +1.0;
    gp_Vec pivotOffset = uVec * pivotU;   // 轴线端点 -> 切面枢轴边框端部

    gp_Vec T(tangent);
    gp_Pnt extStart = item.sweepStart;
    gp_Pnt extEnd   = item.sweepEnd;

    // 原始路径
    SweepPathData extPath;
    SweepPathSeg eseg;
    eseg.isArc = false;
    eseg.start = extStart;
    eseg.end   = extEnd;
    extPath.segments.push_back(eseg);
    SweepPathData orderedExt;
    if (!ValidateSweepPath(extPath) || !OrderSweepPathSegments(extPath, orderedExt))
    {
        std::cout << tag << "路径构造失败" << std::endl;
        return TopoDS_Shape();
    }

    // 截面锚点保持在原始起点（U/V 姿态不变）
    profile.planeOrigin = extStart;
    TopoDS_Wire spineWire   = BuildWireFromSweepPath(orderedExt);
    TopoDS_Wire sectionWire = BuildWireFromSection(profile);
    if (spineWire.IsNull() || sectionWire.IsNull())
    {
        std::cout << tag << "延长扫掠 wire 构造失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Shape extShell = BuildSweepShell(spineWire, sectionWire);
    if (extShell.IsNull())
    {
        std::cout << tag << "延长扫掠壳生成失败" << std::endl;
        return TopoDS_Shape();
    }

    // 封闭壳 -> 实体（布尔求交需要实体参与，斜切面才会被自动补成实体面；
    // 若用壳做布尔，只会裁掉部分壳而缺少斜切面）
    if (extShell.ShapeType() != TopAbs_SHELL)
    {
        std::cout << tag << "延长扫掠结果不是壳，无法实体化切角" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Shell extShellTyped = TopoDS::Shell(extShell);
    BRepBuilderAPI_MakeSolid solidMaker(extShellTyped);
    if (!solidMaker.IsDone())
    {
        std::cout << tag << "扫掠壳实体化失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Solid solid = solidMaker.Solid();

    // 轴线上的中点：两个半空间都以它作为"保留侧"参考点
    gp_Pnt midPoint = item.sweepStart.Translated(T * item.sweepStart.Distance(item.sweepEnd) * 0.5);

    // 单个端部切角：构造过枢轴点、外法向旋转后的半空间并求交。
    // 旋转轴 = 扫掠平面法向 n_sweep（路径方向与截面横向 U 组成平面的法线）。
    // baseNormal：终点用 +T，起点用 -T；axisPoint：斜面枢轴点（边框端部边缘）。
    // rotSign 由 g_invertedTrapezoidCut 选择：梯形 终点-1/起点+1，
    //          倒梯形 终点+1/起点-1——两端切面均向内倾斜。
    auto cutEnd = [&](double angleDeg, const gp_Pnt& axisPoint,
                      const gp_Vec& baseNormal, double rotSign) {
        if (fabs(angleDeg) < 1.0e-9)
            return true;   // 角度0：不切
        double rad = AXO_PI / 180.0 * angleDeg * rotSign;
        gp_Trsf rot;
        rot.SetRotation(gp_Ax1(axisPoint, gp_Dir(item.sweepPlaneNormal)), rad);
        gp_Vec nv = baseNormal;
        nv.Transform(rot);
        gp_Pln cutPlane(axisPoint, gp_Dir(nv));
        TopoDS_Face planeFace = BRepBuilderAPI_MakeFace(cutPlane);
        BRepPrimAPI_MakeHalfSpace halfMaker(planeFace, midPoint);
        if (!halfMaker.IsDone())
            return false;
        BRepAlgoAPI_Common commoner(solid, halfMaker.Solid());
        if (!commoner.IsDone() || commoner.Shape().IsNull())
            return false;
        TopoDS_Shape r = commoner.Shape();
        // 提取布尔结果中的实体（结果可能被包在 Compound 中）
        if (r.ShapeType() == TopAbs_SOLID)
            solid = TopoDS::Solid(r);
        else
        {
            for (TopExp_Explorer ex(r, TopAbs_SOLID); ex.More(); ex.Next())
            {
                solid = TopoDS::Solid(ex.Current());
                break;
            }
        }
        return true;
    };

    if (!cutEnd(item.endAng, item.sweepEnd.Translated(pivotOffset), T, endRotSign))
    {
        std::cout << tag << "终点切角布尔运算失败（角度=" << item.endAng << "）" << std::endl;
        return TopoDS_Shape();
    }
    if (!cutEnd(item.startAng, item.sweepStart.Translated(pivotOffset), gp_Vec(tangent.Reversed()), startRotSign))
    {
        std::cout << tag << "起点切角布尔运算失败（角度=" << item.startAng << "）" << std::endl;
        return TopoDS_Shape();
    }

    // 切角后修复：重建 3D 曲线（布尔结果边几何需对 HLR 完备）
    BRepLib::BuildCurves3d(solid);
    {
        Handle(ShapeFix_Shape) cutFix = new ShapeFix_Shape(solid);
        cutFix->Perform();
        TopoDS_Shape fixed = cutFix->Shape();
        if (!fixed.IsNull() && fixed.ShapeType() == TopAbs_SOLID)
            solid = TopoDS::Solid(fixed);
    }
    BRepLib::BuildCurves3d(solid);

    BRepCheck_Analyzer cutChecker(solid);
    if (!cutChecker.IsValid())
        std::cout << tag << "警告：切角后实体拓扑检查未通过（仍输出供观察）" << std::endl;

    std::cout << tag << "切角完成（起点" << item.startAng << "度/终点" << item.endAng
              << "度）" << std::endl;
    return solid;
}

// 投影算法开关：false = 原 HLRBRep_Algo 解析投影流程（直线/圆弧/圆 + 离散补充）；
//               true  = HLRBRep_PolyAlgo 多面体消隐流程（BRepMesh 三角化，全折线输出）。
// 仅切换投影计算与 DXF 输出文件，建模/脊线/视图变换等其余流程完全不变。
static const bool g_usePolyHLR = false;

// JSON 线框扫掠测试：
// 加载 txt 中的 JSON 扫掠定义，逐条生成扫掠体并汇总为一个 Compound。
// 返回值：0=成功（resultModel 为汇总模型）；-1=失败（resultModel 置空）。
int JsonWireSweepTest(const char* file, TopoDS_Shape& resultModel)
{
    std::cout << "\n--- JSON 线框扫掠测试 ---" << std::endl;
    resultModel = TopoDS_Shape();

    // 1. 加载 + 解析
    std::vector<JsonSweepItem> items;
    if (!LoadJsonSweepItems(file, items))
        return -1;

    // 2. 逐条建模并汇总
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (size_t i = 0; i < items.size(); ++i)
    {
        TopoDS_Shape solid = BuildSolidFromJsonItem(items[i], i + 1);
        if (solid.IsNull())
        {
            std::cout << "错误：第" << (i + 1) << " 条扫掠定义建模失败，终止" << std::endl;
            return -1;
        }
        builder.Add(compound, solid);
    }
    resultModel = compound;

    // 3. 汇总状态信息：拓扑合法性 + 包围盒（供后续验证）
    BRepCheck_Analyzer checker(resultModel);
    Bnd_Box box;
    BRepBndLib::Add(resultModel, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    std::cout << "JSON 线框扫掠模型生成完成：" << items.size() << " 个扫掠体，拓扑"
              << (checker.IsValid() ? "合法" : "存在问题")
              << "，整体范围 X[" << xmin << "," << xmax << "] Y[" << ymin << "," << ymax
              << "] Z[" << zmin << "," << zmax << "]" << std::endl;

    // 4. HLR 投影（轴测视线：斜向观察立面框架，viewDir 为 eye->scene 方向）
    const gp_Dir viewDir(1.0, -2.0, 1.0);
    const gp_Dir viewUp(0.0, 0.0, 1.0);
    std::vector<TopoDS_Shape> shapes;
    shapes.push_back(resultModel);

    // 4a/4b. 模型投影（开关切换，不影响建模与脊线流程）
    std::vector<ProjectedCurve2D> curves2d;       // 解析流程输出
    std::vector<ProjectedPolyline2D> polylines;   // 离散/PolyHLR 流程输出
    if (g_usePolyHLR)
    {
        // 多面体消隐：BRepMesh 三角化 + HLRBRep_PolyAlgo，全折线输出
        if (!ProjectShapeTo2DByPolyHLR(shapes, viewDir, viewUp, polylines))
        {
            std::cout << "错误：JSON 扫掠模型 HLRBRep_PolyAlgo 投影计算失败" << std::endl;
            return -1;
        }
        std::cout << "HLRBRep_PolyAlgo 多面体投影完成：折线数=" << polylines.size()
                  << "（视线(" << viewDir.X() << "," << viewDir.Y() << "," << viewDir.Z()
                  << ")，上方向(" << viewUp.X() << "," << viewUp.Y() << "," << viewUp.Z()
                  << ")）" << std::endl;
    }
    else
    {
        // 4a. 解析投影：可见边按 直线/圆弧/圆 原样输出，不离散
        if (!ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, curves2d))
        {
            std::cout << "错误：JSON 扫掠模型 HLR 解析投影计算失败" << std::endl;
            return -1;
        }
        std::cout << "HLR 解析投影完成：曲线数=" << curves2d.size()
                  << "（视线(" << viewDir.X() << "," << viewDir.Y() << "," << viewDir.Z()
                  << ")，上方向(" << viewUp.X() << "," << viewUp.Y() << "," << viewUp.Z()
                  << ")）" << std::endl;

        // 4b. 离散补充投影：解析不支持的曲线边（BSpline 等）离散为折线
        ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, polylines);
        std::cout << "离散补充投影完成：折线数=" << polylines.size() << std::endl;
    }

    // 4c. 脊线投影：必须通过 HLR 同一"世界系->视图系"轴测变换（BuildViewTransform），
    //     直接对点做 p.Transform(trsf) 取 X/Y——与模型可见边的 HLR 输出坐标同源，
    //     红色脊线与扫掠模型的扫掠棱线严格重合（手动点积投影在 viewUp 不垂直于
    //     viewDir 时会错位）。不做遮挡判定：实体内部路径同样完整显示。
    gp_Trsf viewTrsf;
    if (!BuildViewTransform(viewDir, viewUp, viewTrsf))
    {
        std::cout << "错误：脊线视图变换构造失败" << std::endl;
        return -1;
    }
    std::vector<ProjectedPolyline2D> spinePolylines;
    auto projectPoint = [&](const gp_Pnt& p) {
        gp_Pnt q = p;
        q.Transform(viewTrsf);
        return gp_Pnt2d(q.X(), q.Y());
    };
    for (const auto& item : items)
    {
        ProjectedPolyline2D sp;
        // 当前 JSON 路径为单段 Segment（2 点）；将来扩展圆弧/复杂路径时，
        // 在此向 sp.points 追加采样点即可，写出侧无需改动。
        sp.points.push_back(projectPoint(item.sweepStart));
        sp.points.push_back(projectPoint(item.sweepEnd));
        spinePolylines.push_back(sp);
    }
    std::cout << "脊线投影完成：脊线数=" << spinePolylines.size()
              << "（SPINE图层，红色虚线，0.50mm）" << std::endl;

    // 5. 保存 DXF（解析曲线 + 折线合并写出，脊线写入独立 SPINE 图层）。
    //    PolyHLR 分支使用独立文件名，不覆盖原流程输出。
    const char* dxfPath = g_usePolyHLR ? "sweep_json_wire_poly.dxf"
                                      : "sweep_json_wire.dxf";
    if (!WriteAxonometricCurvesToDXF(curves2d, polylines, dxfPath, &spinePolylines,
                                     g_usePolyHLR))   // PolyHLR：允许直线折线写出
    {
        std::cout << "错误：JSON 扫掠模型投影写入 DXF 失败: " << dxfPath << std::endl;
        return -1;
    }
    std::cout << "DXF 已保存: " << dxfPath << std::endl;

    return 0;
}

// ============================================================================
// JSON 线框综合测试（Prism 棱柱 + Sweep 扫掠混合条目，参见 test4.txt）
// ============================================================================

// JSON 棱柱定义：闭合轮廓折线 + 拉伸方向 + 拉伸厚度
// 数据来源：test4.txt 中 {"Prism":{"Outline":[{PlaneNormal},{Segment}...],"Thickness":..}}
// PlaneNormal 仅作"方向"使用，已归一化为 gp_Dir；Thickness 为数值型拉伸长度。
struct JsonPrismItem
{
    std::vector<gp_Pnt> outlinePts; // 轮廓顶点（首末点重合=闭合）
    gp_Dir planeNormal;             // 拉伸方向（已归一化）
    double thickness;               // 拉伸厚度
};

// JSON 线框条目标签：标识该条目是棱柱、扫掠、包围盒还是其他类型
enum class JsonWireKind
{
    Prism,
    Sweep,
    Box,     // 包围盒：3 个正交方向向量 + 3 向尺寸 + 最小角点
    Other
};

// JSON 包围盒定义（test4.txt 中 BoxDirX/Y/Z + BoxSize + MinimumPoint 顶层对象）
// 数据来源：{"BoxDirX":{X,Y,Z},"BoxDirY":{..},"BoxDirZ":{..},
//            "BoxSize":{X,Y,Z},"MinimumPoint":{X,Y,Z}}
// 三个方向向量需彼此正交（实际数据含 ~1e-16 数值噪声，按 1e-6 容忍判定）；
// BoxSize 三个分量必须 > 0；MinimumPoint 为包围盒"最小角"（local frame 原点）。
struct JsonBoxItem
{
    gp_Dir boxDirX;       // X 轴方向（已归一化）
    gp_Dir boxDirY;       // Y 轴方向（已归一化）
    gp_Dir boxDirZ;       // Z 轴方向（已归一化）
    double sizeX, sizeY, sizeZ;  // 三向尺寸（>0）
    gp_Pnt minimumPoint;  // 包围盒最小角点（local 原点）
    // 解析时记录的正交性诊断（dotX_Y, dotX_Z, dotY_Z；理论上=0，实际含噪声）
    double dotXY, dotXZ, dotYZ;
    bool orthoWarn;       // 正交性偏差超 1e-6 时为 true（仍可建模，仅告警）
};

// JSON 线框条目统一容器
struct JsonWireItem
{
    JsonWireKind kind;
    JsonPrismItem prism;   // kind==Prism 时有效
    JsonSweepItem sweep;   // kind==Sweep 时有效
    JsonBoxItem   box;     // kind==Box 时有效
    std::string kindName;  // 调试用：原始类型名
};

// 从 txt 文件加载混合 JSON 线框定义（Prism + Sweep + 其他忽略）。
// 与 LoadJsonSweepItems 区别：本函数按顶层键分派，支持同一文件混合多种条目。
// 错误处理：文件不存在/读取失败/JSON 语法错误/结构或字段非法，均输出原因并返回 false。
static bool LoadJsonWireItems(const char* file, std::vector<JsonWireItem>& items)
{
    items.clear();

    // 1. 读取文件全部内容（二进制方式，随后手动去 BOM）
    std::ifstream fin(file, std::ios::binary);
    if (!fin.is_open())
    {
        std::cout << "错误：无法打开JSON数据文件: " << file << std::endl;
        return false;
    }
    std::stringstream ss;
    ss << fin.rdbuf();
    if (fin.bad())
    {
        std::cout << "错误：JSON数据文件读取失败: " << file << std::endl;
        return false;
    }
    std::string text = ss.str();
    if (text.size() >= 3 &&
        (unsigned char)text[0] == 0xEF &&
        (unsigned char)text[1] == 0xBB &&
        (unsigned char)text[2] == 0xBF)
        text.erase(0, 3);

    // 2. JSON 解析（nlohmann/json）
    nlohmann::json root;
    try
    {
        root = nlohmann::json::parse(text);
    }
    catch (const nlohmann::json::parse_error& e)
    {
        std::cout << "错误：JSON格式错误: " << e.what() << std::endl;
        return false;
    }
    if (!root.is_array())
    {
        std::cout << "错误：JSON顶层必须是数组" << std::endl;
        return false;
    }

    // 3. 逐条解析：按顶层键分派 Prism / Sweep / Other
    int skipped = 0;
    for (size_t idx = 0; idx < root.size(); ++idx)
    {
        const nlohmann::json& elem = root[idx];
        JsonWireItem item;
        item.kind = JsonWireKind::Other;
        item.kindName = "Other";
        try
        {
            if (elem.contains("Prism"))
            {
                item.kind = JsonWireKind::Prism;
                item.kindName = "Prism";
                const nlohmann::json& prism = elem["Prism"];
                if (!prism.contains("Outline") || !prism["Outline"].is_array())
                    throw std::runtime_error("Prism 缺少 Outline 数组");
                if (!prism.contains("Thickness") || !prism["Thickness"].is_number())
                    throw std::runtime_error("Prism 缺少 Thickness 数值");

                item.prism.thickness = prism["Thickness"].get<double>();
                if (item.prism.thickness <= 0.0)
                    throw std::runtime_error("Prism.Thickness 必须 > 0");

                bool gotNormal = false, gotSeg = false;
                for (const auto& sub : prism["Outline"])
                {
                    if (sub.contains("PlaneNormal"))
                    {
                        // Outline[0] 携带 PlaneNormal：拉伸方向（含模长信息，
                        // 但语义仅取方向；模长 ≠ thickness 是数据冗余，归一化处理）
                        gp_Pnt n;
                        if (!ParseJsonPoint(sub["PlaneNormal"], n, "Prism.PlaneNormal"))
                            throw std::runtime_error("Prism.PlaneNormal 非法");
                        gp_Vec nv(n.X(), n.Y(), n.Z());
                        if (nv.Magnitude() < 1.0e-9)
                            throw std::runtime_error("Prism.PlaneNormal 为零向量");
                        item.prism.planeNormal = gp_Dir(nv);
                        gotNormal = true;
                    }
                    else if (sub.contains("Type") && sub["Type"].get<std::string>() == "Segment")
                    {
                        // Segment 折线段：首段端点全部入列；后续段只追加 End
                        // （End 应与前段 End 重合，否则告警但容忍轻微偏差）
                        if (!sub.contains("Start") || !sub.contains("End"))
                            throw std::runtime_error("Prism.Segment 缺少 Start/End");
                        gp_Pnt a, b;
                        if (!ParseJsonPoint(sub["Start"], a, "Prism.Segment.Start") ||
                            !ParseJsonPoint(sub["End"],   b, "Prism.Segment.End"))
                            throw std::runtime_error("Prism.Segment 端点非法");
                        if (item.prism.outlinePts.empty())
                            item.prism.outlinePts.push_back(a);
                        else if (item.prism.outlinePts.back().Distance(a) > 1.0e-6)
                        {
                            std::cout << "警告：第" << (idx + 1) << "条棱柱段连接不连续"
                                      << "（前段End与下段Start 距离="
                                      << item.prism.outlinePts.back().Distance(a)
                                      << "），按段内 Start/End 顺序继续构造" << std::endl;
                        }
                        item.prism.outlinePts.push_back(b);
                        gotSeg = true;
                    }
                }
                if (!gotNormal) throw std::runtime_error("Prism 缺少 PlaneNormal");
                if (!gotSeg)     throw std::runtime_error("Prism 缺少 Segment");
                if (item.prism.outlinePts.size() < 4)
                    throw std::runtime_error("Prism 轮廓顶点不足(闭合轮廓至少4点)");
                if (item.prism.outlinePts.front().Distance(item.prism.outlinePts.back()) > 1.0e-6)
                    throw std::runtime_error("Prism 轮廓不闭合（首末点距离>1e-6）");
            }
            else if (elem.contains("Section") && elem.contains("Sweep"))
            {
                // 预检 Section 是否使用 PolyLn 格式（当前 BuildSolidFromJsonItem 仅支持 PolyLn）。
                // 部分 Sweep 条目用 Segment/Arc 描述截面，当前不支持——按 Other 跳过，
                // 不影响其他 Prism/Sweep/Box 条目的处理（隔离执行）。
                bool hasPolyLn = false;
                for (const auto& sub : elem["Section"])
                {
                    if (sub.contains("Type") && sub["Type"].get<std::string>() == "PolyLn")
                    {
                        hasPolyLn = true;
                        break;
                    }
                }
                if (!hasPolyLn)
                {
                    item.kind = JsonWireKind::Other;
                    item.kindName = "Other";
                    std::cout << "提示：第" << (idx + 1) << " 条 Sweep 的 Section 仅含 Segment/Arc"
                              << "（无 PolyLn，当前不支持），跳过" << std::endl;
                }
                else
                {
                    item.kind = JsonWireKind::Sweep;
                    item.kindName = "Sweep";
                    JsonSweepItem& s = item.sweep;
                    // 复用 LoadJsonSweepItems 内联解析逻辑：Section + Sweep 两段式
                    bool gotSecNormal = false, gotPoly = false;
                    for (const auto& sub : elem["Section"])
                    {
                        if (sub.contains("PlaneNormal"))
                        {
                            gp_Pnt n;
                            if (!ParseJsonPoint(sub["PlaneNormal"], n, "Section.PlaneNormal"))
                                throw std::runtime_error("Section.PlaneNormal 非法");
                            gp_Vec nv(n.X(), n.Y(), n.Z());
                            if (nv.Magnitude() < 1.0e-9)
                                throw std::runtime_error("Section.PlaneNormal 为零向量");
                            s.sectionNormal = gp_Dir(nv);
                            gotSecNormal = true;
                        }
                        else if (sub.contains("Type") && sub["Type"].get<std::string>() == "PolyLn")
                        {
                            // Pt 键按下标数值排序（字典序会使 Pt10 排在 Pt2 前）
                            std::vector<std::pair<int, gp_Pnt2d>> pts;
                            for (auto it = sub.begin(); it != sub.end(); ++it)
                            {
                                const std::string& key = it.key();
                                if (key.size() > 2 && key[0] == 'P' && key[1] == 't' &&
                                    std::isdigit((unsigned char)key[2]))
                                {
                                    int pi = std::stoi(key.substr(2));
                                    gp_Pnt p;
                                    if (!ParseJsonPoint(it.value(), p, key))
                                        throw std::runtime_error(key + " 非法");
                                    pts.push_back(std::make_pair(pi, gp_Pnt2d(p.X(), p.Y())));
                                }
                            }
                            std::sort(pts.begin(), pts.end(),
                                      [](const std::pair<int, gp_Pnt2d>& a,
                                         const std::pair<int, gp_Pnt2d>& b) { return a.first < b.first; });
                            for (const auto& kv : pts)
                                s.sectionPts.push_back(kv.second);
                            gotPoly = !s.sectionPts.empty();
                        }
                    }
                    if (!gotSecNormal) throw std::runtime_error("Section 缺少 PlaneNormal");
                    if (!gotPoly)      throw std::runtime_error("Section 缺少 PolyLn 顶点");

                    bool gotSwpNormal = false, gotSeg = false;
                    s.startAng = 0.0;
                    s.endAng = 0.0;
                    for (const auto& sub : elem["Sweep"])
                    {
                        if (sub.contains("PlaneNormal"))
                        {
                            gp_Pnt n;
                            if (!ParseJsonPoint(sub["PlaneNormal"], n, "Sweep.PlaneNormal"))
                                throw std::runtime_error("Sweep.PlaneNormal 非法");
                            gp_Vec nv(n.X(), n.Y(), n.Z());
                            if (nv.Magnitude() < 1.0e-9)
                                throw std::runtime_error("Sweep.PlaneNormal 为零向量");
                            s.sweepPlaneNormal = gp_Dir(nv);
                            gotSwpNormal = true;
                        }
                        else if (sub.contains("Type") && sub["Type"].get<std::string>() == "Segment")
                        {
                            if (!sub.contains("Start") || !sub.contains("End"))
                                throw std::runtime_error("Segment 缺少 Start/End");
                            if (!ParseJsonPoint(sub["Start"], s.sweepStart, "Sweep.Start") ||
                                !ParseJsonPoint(sub["End"],   s.sweepEnd,   "Sweep.End"))
                                throw std::runtime_error("Segment 端点非法");
                            gotSeg = true;
                        }
                        else if (sub.contains("StartAng") || sub.contains("EndAng"))
                        {
                            // 切角角度（度）：0=不切；字段缺失按 0 处理
                            if (sub.contains("StartAng")) s.startAng = sub["StartAng"].get<double>();
                            if (sub.contains("EndAng"))   s.endAng   = sub["EndAng"].get<double>();
                            if (fabs(s.startAng) >= 90.0 || fabs(s.endAng) >= 90.0)
                                throw std::runtime_error("切角角度必须在(-90,90)范围内");
                        }
                    }
                    if (!gotSwpNormal) throw std::runtime_error("Sweep 缺少 PlaneNormal");
                    if (!gotSeg)       throw std::runtime_error("Sweep 缺少 Segment");

                    if (s.sectionPts.size() < 3)
                        throw std::runtime_error("截面多段线顶点不足(至少需要3个)");
                    if (s.sectionPts.front().Distance(s.sectionPts.back()) > 1.0e-6)
                        throw std::runtime_error("截面多段线不闭合（首末点距离>1e-6）");
                    if (s.sweepStart.Distance(s.sweepEnd) < 1.0e-9)
                        throw std::runtime_error("扫掠路径退化（起点终点重合）");
                }   // else (hasPolyLn) 块结束
            }
            else if (elem.contains("Box"))    // 包围盒：{"Box":{"BoxDirX":..,"BoxDirY":..,"BoxDirZ":..,"BoxSize":..,"MinimumPoint":..}}
            {
                // Box：5 个必填属性（全在 "Box" 包装对象内）
                item.kind = JsonWireKind::Box;
                item.kindName = "Box";
                JsonBoxItem& b = item.box;
                b.dotXY = b.dotXZ = b.dotYZ = 0.0;
                b.orthoWarn = false;

                // 取包装对象，内部键为 BoxDirX/Y/Z + BoxSize + MinimumPoint
                const nlohmann::json& boxObj = elem["Box"];
                if (!boxObj.is_object())
                    throw std::runtime_error("Box 不是对象");

                // BoxDirX/Y/Z + BoxSize + MinimumPoint 都使用大写 X/Y/Z 键名，
                // 用大小写不敏感解析器；MinimumPoint 接受规范拼写也容忍错别字
                // "MinimunPoint"（用户文档中出现的拼写变体）。
                if (!boxObj.contains("BoxDirX")) throw std::runtime_error("Box 缺少 BoxDirX");
                if (!boxObj.contains("BoxDirY")) throw std::runtime_error("Box 缺少 BoxDirY");
                if (!boxObj.contains("BoxDirZ")) throw std::runtime_error("Box 缺少 BoxDirZ");
                if (!boxObj.contains("BoxSize")) throw std::runtime_error("Box 缺少 BoxSize");

                gp_Pnt dx, dy, dz, sz, mn;
                if (!ParseJsonPointCI(boxObj["BoxDirX"], dx, "BoxDirX"))
                    throw std::runtime_error("Box.BoxDirX 非法");
                if (!ParseJsonPointCI(boxObj["BoxDirY"], dy, "BoxDirY"))
                    throw std::runtime_error("Box.BoxDirY 非法");
                if (!ParseJsonPointCI(boxObj["BoxDirZ"], dz, "BoxDirZ"))
                    throw std::runtime_error("Box.BoxDirZ 非法");
                if (!ParseJsonPointCI(boxObj["BoxSize"], sz, "BoxSize"))
                    throw std::runtime_error("Box.BoxSize 非法");
                if (boxObj.contains("MinimumPoint"))
                {
                    if (!ParseJsonPointCI(boxObj["MinimumPoint"], mn, "MinimumPoint"))
                        throw std::runtime_error("Box.MinimumPoint 非法");
                }
                else if (boxObj.contains("MinimunPoint"))
                {
                    // 容忍规范文档中出现的错别字"MinimunPoint"
                    if (!ParseJsonPointCI(boxObj["MinimunPoint"], mn, "MinimunPoint"))
                        throw std::runtime_error("Box.MinimunPoint 非法");
                }
                else
                    throw std::runtime_error("Box 缺少 MinimumPoint 数组");

                // 方向向量模长非零（归一化后存）
                gp_Vec vx(dx.X(), dx.Y(), dx.Z());
                gp_Vec vy(dy.X(), dy.Y(), dy.Z());
                gp_Vec vz(dz.X(), dz.Y(), dz.Z());
                if (vx.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirX 为零向量");
                if (vy.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirY 为零向量");
                if (vz.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirZ 为零向量");
                b.boxDirX = gp_Dir(vx);
                b.boxDirY = gp_Dir(vy);
                b.boxDirZ = gp_Dir(vz);

                // 三向尺寸必须为正
                b.sizeX = sz.X();
                b.sizeY = sz.Y();
                b.sizeZ = sz.Z();
                if (b.sizeX <= 0.0 || b.sizeY <= 0.0 || b.sizeZ <= 0.0)
                    throw std::runtime_error("BoxSize 三向尺寸必须 > 0");
                b.minimumPoint = mn;

                // 正交性诊断：理论 dot=0；实际数据含 ~1e-16 噪声，1e-6 容忍
                b.dotXY = gp_Vec(b.boxDirX).Dot(gp_Vec(b.boxDirY));
                b.dotXZ = gp_Vec(b.boxDirX).Dot(gp_Vec(b.boxDirZ));
                b.dotYZ = gp_Vec(b.boxDirY).Dot(gp_Vec(b.boxDirZ));
                const double orthoTol = 1.0e-6;
                if (fabs(b.dotXY) > orthoTol || fabs(b.dotXZ) > orthoTol || fabs(b.dotYZ) > orthoTol)
                {
                    b.orthoWarn = true;
                    std::cout << "警告：第" << (idx + 1) << "条 Box 方向向量非正交："
                              << "dotXY=" << b.dotXY << " dotXZ=" << b.dotXZ
                              << " dotYZ=" << b.dotYZ << "（仍按给定方向建模）" << std::endl;
                }
            }
            else if (elem.contains("BoxDirX") && elem.contains("BoxDirY") &&
                     elem.contains("BoxDirZ") && elem.contains("BoxSize"))
            {
                // 兼容：未包装的扁平格式 {"BoxDirX":..,...}（数据冗余场合）
                item.kind = JsonWireKind::Box;
                item.kindName = "Box";
                JsonBoxItem& b = item.box;
                b.dotXY = b.dotXZ = b.dotYZ = 0.0;
                b.orthoWarn = false;

                gp_Pnt dx, dy, dz, sz, mn;
                if (!ParseJsonPointCI(elem["BoxDirX"], dx, "BoxDirX"))
                    throw std::runtime_error("Box.BoxDirX 非法");
                if (!ParseJsonPointCI(elem["BoxDirY"], dy, "BoxDirY"))
                    throw std::runtime_error("Box.BoxDirY 非法");
                if (!ParseJsonPointCI(elem["BoxDirZ"], dz, "BoxDirZ"))
                    throw std::runtime_error("Box.BoxDirZ 非法");
                if (!ParseJsonPointCI(elem["BoxSize"], sz, "BoxSize"))
                    throw std::runtime_error("Box.BoxSize 非法");
                if (elem.contains("MinimumPoint"))
                {
                    if (!ParseJsonPointCI(elem["MinimumPoint"], mn, "MinimumPoint"))
                        throw std::runtime_error("Box.MinimumPoint 非法");
                }
                else if (elem.contains("MinimunPoint"))
                {
                    if (!ParseJsonPointCI(elem["MinimunPoint"], mn, "MinimunPoint"))
                        throw std::runtime_error("Box.MinimunPoint 非法");
                }
                else
                    throw std::runtime_error("Box 缺少 MinimumPoint 数组");

                gp_Vec vx(dx.X(), dx.Y(), dx.Z());
                gp_Vec vy(dy.X(), dy.Y(), dy.Z());
                gp_Vec vz(dz.X(), dz.Y(), dz.Z());
                if (vx.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirX 为零向量");
                if (vy.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirY 为零向量");
                if (vz.Magnitude() < 1.0e-9)
                    throw std::runtime_error("BoxDirZ 为零向量");
                b.boxDirX = gp_Dir(vx);
                b.boxDirY = gp_Dir(vy);
                b.boxDirZ = gp_Dir(vz);

                b.sizeX = sz.X();
                b.sizeY = sz.Y();
                b.sizeZ = sz.Z();
                if (b.sizeX <= 0.0 || b.sizeY <= 0.0 || b.sizeZ <= 0.0)
                    throw std::runtime_error("BoxSize 三向尺寸必须 > 0");
                b.minimumPoint = mn;

                b.dotXY = gp_Vec(b.boxDirX).Dot(gp_Vec(b.boxDirY));
                b.dotXZ = gp_Vec(b.boxDirX).Dot(gp_Vec(b.boxDirZ));
                b.dotYZ = gp_Vec(b.boxDirY).Dot(gp_Vec(b.boxDirZ));
                const double orthoTol2 = 1.0e-6;
                if (fabs(b.dotXY) > orthoTol2 || fabs(b.dotXZ) > orthoTol2 || fabs(b.dotYZ) > orthoTol2)
                {
                    b.orthoWarn = true;
                    std::cout << "警告：第" << (idx + 1) << "条 Box 方向向量非正交："
                              << "dotXY=" << b.dotXY << " dotXZ=" << b.dotXZ
                              << " dotYZ=" << b.dotYZ << "（仍按给定方向建模）" << std::endl;
                }
            }
            else
            {
                // 其他未知类型：记录键并跳过
                item.kind = JsonWireKind::Other;
                item.kindName = "Other";
                std::string keys;
                for (auto it = elem.begin(); it != elem.end(); ++it)
                {
                    if (!keys.empty()) keys += ",";
                    keys += it.key();
                }
                std::cout << "提示：第" << (idx + 1) << " 条为非 Prism/Sweep/Box 类型 ["
                          << keys << "]，跳过" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "JSON错误：第" << (idx + 1) << " 条定义无效：" << e.what() << std::endl;
            return false;
        }
        if (item.kind == JsonWireKind::Other)
            ++skipped;
        else
            items.push_back(item);
    }

    if (items.empty())
    {
        std::cout << "错误：JSON中没有任何 Prism/Sweep/Box 定义" << std::endl;
        return false;
    }
    std::cout << "JSON数据读取完成: " << file << "，共 " << items.size()
              << " 条线框定义（跳过" << skipped << "条非 Prism/Sweep/Box）" << std::endl;
    return true;
}

// 单条 JSON 棱柱定义 -> 拉伸实体（轮廓构建面 + 沿方向拉伸 Thickness）
// 流程：多段线 wire -> BuildFaceWithHoles（含 ShapeFix_Face::FixOrientation 自动定向）
//      -> BRepPrimAPI_MakePrism 沿 PlaneNormal*thickness 拉伸 -> BuildCurves3d
static TopoDS_Shape BuildPrismSolid(const JsonPrismItem& item, size_t index)
{
    const std::string tag = "第" + std::to_string(index) + "条棱柱定义：";

    // 1. 构建外轮廓 wire（多段折线）
    BRepBuilderAPI_MakePolygon polyMaker;
    for (const auto& p : item.outlinePts)
        polyMaker.Add(p);
    polyMaker.Close();
    if (!polyMaker.IsDone())
    {
        std::cout << tag << "轮廓 wire 构造失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Wire outerWire = polyMaker.Wire();
    if (outerWire.IsNull())
    {
        std::cout << tag << "轮廓 wire 为空" << std::endl;
        return TopoDS_Shape();
    }

    // 2. 在过轮廓起点的平面（法向 = PlaneNormal）上构造面
    //    BuildFaceWithHoles 通过 ShapeFix_Face::FixOrientation 自动修正 wire 方向，
    //    调用方不必保证 wire 在平面 UV 空间的环绕方向。
    gp_Pnt planeOrigin = item.outlinePts.front();
    std::vector<TopoDS_Wire> noHoles;
    TopoDS_Face baseFace = BuildFaceWithHoles(outerWire, noHoles,
                                              planeOrigin, item.planeNormal, 1.0e-6);
    if (baseFace.IsNull())
    {
        std::cout << tag << "底面构造失败（BuildFaceWithHoles 返回空）" << std::endl;
        return TopoDS_Shape();
    }

    // 3. 沿 PlaneNormal 方向拉伸 Thickness（方向已归一化，乘 thickness 得位移向量）
    gp_Vec extrudeVec(item.planeNormal);
    extrudeVec.Multiply(item.thickness);
    BRepPrimAPI_MakePrism prismMaker(baseFace, extrudeVec);
    if (!prismMaker.IsDone())
    {
        std::cout << tag << "MakePrism 拉伸失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Shape prism = prismMaker.Shape();
    if (prism.IsNull())
    {
        std::cout << tag << "拉伸结果为空" << std::endl;
        return TopoDS_Shape();
    }

    // 4. 重建 3D 曲线（HLR 投影需要边几何对 HLR 完备）
    BRepLib::BuildCurves3d(prism);

    // 5. 拓扑合法性校验（失败仍返回结果供观察）
    BRepCheck_Analyzer checker(prism);
    if (!checker.IsValid())
        std::cout << tag << "警告：拉伸实体拓扑检查未通过（仍输出供观察）" << std::endl;

    return prism;
}

// 单条 JSON 包围盒定义 -> 拉伸实体（底面 4 角 wire + BuildFaceWithHoles +
//                            BRepPrimAPI_MakePrism 沿 BoxDirZ*BoxSize.Z 拉伸）
// 几何约定：minimumPoint 为底面角点；BoxDirX/BoxDirY 决定底面姿态，BoxDirZ 决定拉伸方向；
//           底面 4 角 = mn, mn+X*sx, mn+X*sx+Y*sy, mn+Y*sy（绕 BoxDirZ 右手系）。
static TopoDS_Shape BuildBoxSolid(const JsonBoxItem& item, size_t index)
{
    const std::string tag = "第" + std::to_string(index) + "条包围盒定义：";

    // 1. 计算 4 个底角点（按 BoxDirX × BoxDirY 平面展开）
    gp_Pnt p0 = item.minimumPoint;
    gp_Pnt p1 = p0.Translated(gp_Vec(item.boxDirX) * item.sizeX);
    gp_Pnt p2 = p1.Translated(gp_Vec(item.boxDirY) * item.sizeY);
    gp_Pnt p3 = p0.Translated(gp_Vec(item.boxDirY) * item.sizeY);

    // 2. 构建底面外环 wire
    BRepBuilderAPI_MakePolygon polyMaker;
    polyMaker.Add(p0);
    polyMaker.Add(p1);
    polyMaker.Add(p2);
    polyMaker.Add(p3);
    polyMaker.Close();
    if (!polyMaker.IsDone())
    {
        std::cout << tag << "底面 wire 构造失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Wire outerWire = polyMaker.Wire();
    if (outerWire.IsNull())
    {
        std::cout << tag << "底面 wire 为空" << std::endl;
        return TopoDS_Shape();
    }

    // 3. 在过 p0、法向 = BoxDirZ 的平面上构面（ShapeFix_Face::FixOrientation 自动定向）
    std::vector<TopoDS_Wire> noHoles;
    TopoDS_Face baseFace = BuildFaceWithHoles(outerWire, noHoles,
                                              p0, item.boxDirZ, 1.0e-6);
    if (baseFace.IsNull())
    {
        std::cout << tag << "底面构造失败（BuildFaceWithHoles 返回空）" << std::endl;
        return TopoDS_Shape();
    }

    // 4. 沿 BoxDirZ 方向拉伸 BoxSize.Z
    gp_Vec extrudeVec(item.boxDirZ);
    extrudeVec.Multiply(item.sizeZ);
    BRepPrimAPI_MakePrism prismMaker(baseFace, extrudeVec);
    if (!prismMaker.IsDone())
    {
        std::cout << tag << "MakePrism 拉伸失败" << std::endl;
        return TopoDS_Shape();
    }
    TopoDS_Shape box = prismMaker.Shape();
    if (box.IsNull())
    {
        std::cout << tag << "拉伸结果为空" << std::endl;
        return TopoDS_Shape();
    }

    // 5. 重建 3D 曲线 + 拓扑校验
    BRepLib::BuildCurves3d(box);
    BRepCheck_Analyzer checker(box);
    if (!checker.IsValid())
        std::cout << tag << "警告：包围盒拓扑检查未通过（仍输出供观察）" << std::endl;

    return box;
}

// JSON 线框综合测试：加载 txt 中混合的 Prism/Sweep/Box 定义，逐条建模并汇总。
// - 输入验证：outline 非空且闭合、方向非零、thickness>0；sweep 路径有效、截面合法；
//             box 三向尺寸>0、方向非零、MinimumPoint 含 X/Y/Z
// - 异常处理：每条独立 try/catch，单条失败不影响其他条目
// - 结果断言：BRepCheck_Analyzer 校验拓扑合法性；Box 额外校验体积/包围盒/正交性
// - 测试报告：每条目输出 类型/参数/状态/包围盒，最后输出汇总
// - 可视化：HLR 投影 + DXF 写出（与 JsonWireSweepTest 同一视图变换）
// 返回值：0=成功（resultModel 为汇总模型）；-1=失败（resultModel 置空）

// PerThreadBuf：将 std::cout 重定向到 thread_local 缓冲区，实现多线程下逐线程独立缓冲输出
// 用法：安装到 cout 后，每个线程通过 TakeString() 取走自己线程累积的输出并清空
class PerThreadBuf : public std::streambuf
{
public:
    // 取走当前线程累积的输出并清空缓冲
    std::string TakeString()
    {
        std::string out;
        out.swap(tls_buf);
        return out;
    }
    // 清空当前线程缓冲
    void Clear() { tls_buf.clear(); }
protected:
    std::streamsize xsputn(const char* s, std::streamsize n) override
    {
        tls_buf.append(s, static_cast<size_t>(n));
        return n;
    }
    int_type overflow(int_type c) override
    {
        if (!traits_type::eq_int_type(c, traits_type::eof()))
            tls_buf.push_back(traits_type::to_char_type(c));
        return !traits_type::eof();
    }
private:
    thread_local static std::string tls_buf;   // 每线程独立缓冲
};
thread_local std::string PerThreadBuf::tls_buf;

int JsonWireTest(const char* file, TopoDS_Shape& resultModel)
{
    std::cout << "\n--- JSON 线框综合测试（Prism + Sweep + Box，文件: " << file << "）---" << std::endl;
    resultModel = TopoDS_Shape();

    // 1. 加载 + 解析（顶层键分派 Prism/Sweep/Box/Other）
    std::vector<JsonWireItem> items;
    if (!LoadJsonWireItems(file, items))
        return -1;

    // 2. 逐条建模、报告、汇总（多线程并行，输出按序回放）
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    std::vector<TopoDS_Shape> solids(items.size());
    std::vector<std::string> itemReports(items.size());
    std::atomic<int> prismCount(0), sweepCount(0), boxCount(0), failCount(0);
    std::atomic<int> boxAssertPass(0), boxAssertWarn(0);
    std::atomic<size_t> nextIdx(0);

    // 安装逐线程 cout 缓冲：并行期间所有 cout 输出进入各线程的 thread_local 缓冲
    PerThreadBuf perThreadBuf;
    std::streambuf* oldBuf = std::cout.rdbuf(&perThreadBuf);

    unsigned int nThreads = std::thread::hardware_concurrency();
    if (nThreads == 0) nThreads = 4;
    if (nThreads > items.size()) nThreads = (unsigned int)items.size();

    auto worker = [&]() {
        size_t i;
        while ((i = nextIdx.fetch_add(1)) < items.size())
        {
            perThreadBuf.Clear();
            const JsonWireItem& it = items[i];
            size_t idx = i + 1;
            TopoDS_Shape solid;
            std::cout << "\n[第" << idx << "条] 类型=" << it.kindName;
            try
            {
                if (it.kind == JsonWireKind::Prism)
                {
                    ++prismCount;
                    std::cout << " 顶点数=" << it.prism.outlinePts.size()
                              << " 拉伸方向=(" << it.prism.planeNormal.X() << ","
                              << it.prism.planeNormal.Y() << "," << it.prism.planeNormal.Z() << ")"
                              << " 厚度=" << it.prism.thickness << std::endl;
                    solid = BuildPrismSolid(it.prism, idx);
                }
                else if (it.kind == JsonWireKind::Sweep)
                {
                    ++sweepCount;
                    std::cout << " 截面顶点数=" << it.sweep.sectionPts.size()
                              << " 起点=(" << it.sweep.sweepStart.X() << ","
                              << it.sweep.sweepStart.Y() << "," << it.sweep.sweepStart.Z() << ")"
                              << " 终点=(" << it.sweep.sweepEnd.X() << ","
                              << it.sweep.sweepEnd.Y() << "," << it.sweep.sweepEnd.Z() << ")"
                              << " 切角 起=" << it.sweep.startAng << " 终=" << it.sweep.endAng
                              << std::endl;
                    solid = BuildSolidFromJsonItem(it.sweep, idx);
                }
                else if (it.kind == JsonWireKind::Box)
                {
                    ++boxCount;
                    std::cout << " 方向X=(" << it.box.boxDirX.X() << ","
                              << it.box.boxDirY.Y() << "," << it.box.boxDirZ.Z() << ")"
                              << " 尺寸=(" << it.box.sizeX << "," << it.box.sizeY << ","
                              << it.box.sizeZ << ")"
                              << " 最小点=(" << it.box.minimumPoint.X() << ","
                              << it.box.minimumPoint.Y() << "," << it.box.minimumPoint.Z() << ")"
                              << " 正交性";
                    if (it.box.orthoWarn)
                    {
                        std::cout << "异常(dotXY=" << it.box.dotXY
                                  << ",dotXZ=" << it.box.dotXZ
                                  << ",dotYZ=" << it.box.dotYZ << ")";
                        ++boxAssertWarn;
                    }
                    else
                    {
                        std::cout << "良好";
                        ++boxAssertPass;
                    }
                    std::cout << std::endl;
                    solid = BuildBoxSolid(it.box, idx);
                }
            }
            catch (const Standard_ConstructionError& e)
            {
                std::cout << "  异常：OCCT构造错误：" << e.GetMessageString() << std::endl;
                solid = TopoDS_Shape();
            }
            catch (const std::exception& e)
            {
                std::cout << "  异常：" << e.what() << std::endl;
                solid = TopoDS_Shape();
            }

            if (solid.IsNull())
            {
                std::cout << "  状态：失败" << std::endl;
                ++failCount;
                itemReports[i] = perThreadBuf.TakeString();
                continue;
            }

            // 拓扑合法性 + 实际包围盒
            BRepCheck_Analyzer checker(solid);
            Bnd_Box box;
            BRepBndLib::Add(solid, box);
            double xmin, ymin, zmin, xmax, ymax, zmax;
            box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
            std::cout << "  状态：" << (checker.IsValid() ? "成功(拓扑合法)" : "成功(拓扑异常)")
                      << " bbox X[" << xmin << "," << xmax << "] Y[" << ymin << "," << ymax
                      << "] Z[" << zmin << "," << zmax << "]" << std::endl;

            // === Box 结果断言：体积 + 期望包围盒 ===
            if (it.kind == JsonWireKind::Box)
            {
                // (a) 体积断言：实际体积应等于 sizeX*sizeY*sizeZ（容差 1e-3）
                GProp_GProps volProps;
                BRepGProp::VolumeProperties(solid, volProps);
                double actualVol = volProps.Mass();
                double expectedVol = it.box.sizeX * it.box.sizeY * it.box.sizeZ;
                double volErr = fabs(actualVol - expectedVol);
                std::cout << "  断言[体积]：实际=" << actualVol
                          << " 期望=" << expectedVol
                          << " 误差=" << volErr
                          << (volErr < 1.0e-3 ? " -> 通过" : " -> 失败") << std::endl;

                // (b) 期望包围盒：8 个角点的 AABB，与实际 BrepBndLib 输出对比
                gp_Pnt corners[8];
                gp_Pnt mn = it.box.minimumPoint;
                corners[0] = mn;
                corners[1] = mn.Translated(gp_Vec(it.box.boxDirX) * it.box.sizeX);
                corners[2] = mn.Translated(gp_Vec(it.box.boxDirY) * it.box.sizeY);
                corners[3] = mn.Translated(gp_Vec(it.box.boxDirX) * it.box.sizeX)
                                .Translated(gp_Vec(it.box.boxDirY) * it.box.sizeY);
                // 顶面 4 角（沿 BoxDirZ 拉伸 sizeZ）
                for (int k = 0; k < 4; ++k)
                    corners[k + 4] = corners[k].Translated(gp_Vec(it.box.boxDirZ) * it.box.sizeZ);
                double exMinX = corners[0].X(), exMaxX = corners[0].X();
                double exMinY = corners[0].Y(), exMaxY = corners[0].Y();
                double exMinZ = corners[0].Z(), exMaxZ = corners[0].Z();
                for (int k = 1; k < 8; ++k)
                {
                    if (corners[k].X() < exMinX) exMinX = corners[k].X();
                    if (corners[k].X() > exMaxX) exMaxX = corners[k].X();
                    if (corners[k].Y() < exMinY) exMinY = corners[k].Y();
                    if (corners[k].Y() > exMaxY) exMaxY = corners[k].Y();
                    if (corners[k].Z() < exMinZ) exMinZ = corners[k].Z();
                    if (corners[k].Z() > exMaxZ) exMaxZ = corners[k].Z();
                }
                const double bbTol = 1.0e-3;
                bool bbOK =
                    fabs(xmin - exMinX) < bbTol && fabs(xmax - exMaxX) < bbTol &&
                    fabs(ymin - exMinY) < bbTol && fabs(ymax - exMaxY) < bbTol &&
                    fabs(zmin - exMinZ) < bbTol && fabs(zmax - exMaxZ) < bbTol;
                std::cout << "  断言[包围盒]：期望 X[" << exMinX << "," << exMaxX
                          << "] Y[" << exMinY << "," << exMaxY
                          << "] Z[" << exMinZ << "," << exMaxZ << "]"
                          << (bbOK ? " -> 通过" : " -> 失败") << std::endl;

                // (c) 正交性断言：3 个方向向量两两点积应近似为 0
                const double orthoTol = 1.0e-6;
                bool orthoOK =
                    fabs(it.box.dotXY) < orthoTol &&
                    fabs(it.box.dotXZ) < orthoTol &&
                    fabs(it.box.dotYZ) < orthoTol;
                std::cout << "  断言[正交性]：dotXY=" << it.box.dotXY
                          << " dotXZ=" << it.box.dotXZ
                          << " dotYZ=" << it.box.dotYZ
                          << (orthoOK ? " -> 通过" : " -> 警告(仍建模)")
                          << std::endl;
            }

            solids[i] = solid;
            itemReports[i] = perThreadBuf.TakeString();
        }
    };

    std::vector<std::thread> threads;
    for (unsigned int t = 0; t < nThreads; ++t)
        threads.emplace_back(worker);
    for (auto& t : threads)
        t.join();

    // 恢复 cout，按原始顺序回放各条目报告 + 汇入 compound
    std::cout.rdbuf(oldBuf);
    for (size_t i = 0; i < items.size(); ++i)
    {
        std::cout << itemReports[i];
        if (!solids[i].IsNull())
            builder.Add(compound, solids[i]);
    }

    resultModel = compound;
    std::cout << "\n汇总：Prism=" << prismCount << " Sweep=" << sweepCount
              << " Box=" << boxCount << "(正交性通过" << boxAssertPass
              << "/警告" << boxAssertWarn << ")"
              << " 失败=" << failCount << " 共" << items.size() << "条" << std::endl;
    if (failCount > 0)
        std::cout << "警告：有 " << failCount << " 条建模失败" << std::endl;

    // 3. 汇总包围盒 + 拓扑合法性
    BRepCheck_Analyzer checker(resultModel);
    Bnd_Box box;
    BRepBndLib::Add(resultModel, box);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    std::cout << "JSON 线框模型生成完成：共 " << items.size() << " 个实体，拓扑"
              << (checker.IsValid() ? "合法" : "存在问题")
              << "，整体范围 X[" << xmin << "," << xmax << "] Y[" << ymin << "," << ymax
              << "] Z[" << zmin << "," << zmax << "]" << std::endl;

    // 4. HLR 投影 + DXF 输出（与 JsonWireSweepTest 同一视图变换，便于结果对比）
    const gp_Dir viewDir(1.0, -2.0, 1.0);
    const gp_Dir viewUp(0.0, 0.0, 1.0);
    std::vector<TopoDS_Shape> shapes;
    shapes.push_back(resultModel);

    std::vector<ProjectedCurve2D> curves2d;       // 解析流程输出（下游使用）
    std::vector<ProjectedPolyline2D> polylines;   // 离散/PolyHLR 流程输出（下游使用）

    // 时间统计：同时运行两个 HLR 流程并计时对比效率
    {
        std::vector<ProjectedCurve2D> analyticCurves;   // 解析流程临时输出
        std::vector<ProjectedPolyline2D> analyticSupp; // 解析流程离散补充
        std::vector<ProjectedPolyline2D> polyCurves;   // 多面体流程临时输出

        // 1) 解析流程：ProjectCurveSetTo2DCurves + ProjectBoxSetTo2DCurves
        auto t0 = std::chrono::high_resolution_clock::now();
        ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, analyticCurves);
        ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, analyticSupp);
        auto t1 = std::chrono::high_resolution_clock::now();
        // 2) 多面体流程：ProjectShapeTo2DByPolyHLR
        ProjectShapeTo2DByPolyHLR(shapes, viewDir, viewUp, polyCurves);
        auto t2 = std::chrono::high_resolution_clock::now();

        double analyticMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double polyMs = std::chrono::duration<double, std::milli>(t2 - t1).count();

        // 只输出对比结果
        std::cout << "时间统计：ProjectCurveSetTo2DCurves=" << analyticMs << "ms"
                  << "，ProjectShapeTo2DByPolyHLR=" << polyMs << "ms" << std::endl;
        if (analyticMs <= polyMs)
        {
            std::cout << "对比结果：ProjectCurveSetTo2DCurves 更快（差 " << (polyMs - analyticMs)
                      << "ms，倍率 " << (analyticMs > 0 ? polyMs / analyticMs : 0) << "x）" << std::endl;
        }
        else
        {
            std::cout << "对比结果：ProjectShapeTo2DByPolyHLR 更快（差 " << (analyticMs - polyMs)
                      << "ms，倍率 " << (polyMs > 0 ? analyticMs / polyMs : 0) << "x）" << std::endl;
        }

        // 根据 switch 选择下游使用的结果
        if (g_usePolyHLR)
        {
            polylines.swap(polyCurves);
        }
        else
        {
            curves2d.swap(analyticCurves);
            polylines.swap(analyticSupp);
        }
    }

    // 4c. 脊线投影：仅 Sweep 条目绘制脊线（Prism 无脊线概念）
    gp_Trsf viewTrsf;
    if (!BuildViewTransform(viewDir, viewUp, viewTrsf))
    {
        std::cout << "错误：脊线视图变换构造失败" << std::endl;
        return -1;
    }
    std::vector<ProjectedPolyline2D> spinePolylines;
    auto projectPoint = [&](const gp_Pnt& p) {
        gp_Pnt q = p;
        q.Transform(viewTrsf);
        return gp_Pnt2d(q.X(), q.Y());
    };
    for (const auto& it : items)
    {
        if (it.kind != JsonWireKind::Sweep)
            continue;
        ProjectedPolyline2D sp;
        sp.points.push_back(projectPoint(it.sweep.sweepStart));
        sp.points.push_back(projectPoint(it.sweep.sweepEnd));
        spinePolylines.push_back(sp);
    }
    if (!spinePolylines.empty())
        std::cout << "脊线投影完成：脊线数=" << spinePolylines.size()
                  << "（仅 Sweep 条目，SPINE图层，红色虚线）" << std::endl;

    // 5. 保存 DXF（解析曲线 + 折线 + 脊线，与 JsonWireSweepTest 同一写出格式）
    const char* dxfPath = g_usePolyHLR ? "wire_test_poly.dxf" : "wire_test.dxf";
    if (!WriteAxonometricCurvesToDXF(curves2d, polylines, dxfPath,
                                     spinePolylines.empty() ? NULL : &spinePolylines,
                                     g_usePolyHLR))
    {
        std::cout << "错误：JSON 线框模型投影写入 DXF 失败: " << dxfPath << std::endl;
        return -1;
    }
    std::cout << "DXF 已保存: " << dxfPath << std::endl;

    return 0;
}

int JsonWireTest2(const char* file, TopoDS_Shape& resultModel) {
    resultModel = TopoDS_Shape();

    // 1. 加载 + 解析（顶层键分派 Prism/Sweep/Box/Other）
    std::vector<JsonWireItem> items;
    if (!LoadJsonWireItems(file, items))
        return -1;

    int prismCount = 0;
    int sweepCount = 0;
    int boxCount = 0;
    int boxAssertWarn = 0;
    int boxAssertPass = 0;

    std::vector<TopoDS_Shape> solids(items.size());

    for (size_t i = 0; i < items.size(); i++)
    {
        const JsonWireItem& it = items[i];
        TopoDS_Shape solid;
        try
        {
            if (it.kind == JsonWireKind::Prism)
            {
                ++prismCount;
                std::cout << " 顶点数=" << it.prism.outlinePts.size()
                    << " 拉伸方向=(" << it.prism.planeNormal.X() << ","
                    << it.prism.planeNormal.Y() << "," << it.prism.planeNormal.Z() << ")"
                    << " 厚度=" << it.prism.thickness << std::endl;
                solid = BuildPrismSolid(it.prism, i);
            }
            else if (it.kind == JsonWireKind::Sweep)
            {
                ++sweepCount;
                std::cout << " 截面顶点数=" << it.sweep.sectionPts.size()
                    << " 起点=(" << it.sweep.sweepStart.X() << ","
                    << it.sweep.sweepStart.Y() << "," << it.sweep.sweepStart.Z() << ")"
                    << " 终点=(" << it.sweep.sweepEnd.X() << ","
                    << it.sweep.sweepEnd.Y() << "," << it.sweep.sweepEnd.Z() << ")"
                    << " 切角 起=" << it.sweep.startAng << " 终=" << it.sweep.endAng
                    << std::endl;
                solid = BuildSolidFromJsonItem(it.sweep, i);
            }
            else if (it.kind == JsonWireKind::Box)
            {
                ++boxCount;
                std::cout << " 方向X=(" << it.box.boxDirX.X() << ","
                    << it.box.boxDirY.Y() << "," << it.box.boxDirZ.Z() << ")"
                    << " 尺寸=(" << it.box.sizeX << "," << it.box.sizeY << ","
                    << it.box.sizeZ << ")"
                    << " 最小点=(" << it.box.minimumPoint.X() << ","
                    << it.box.minimumPoint.Y() << "," << it.box.minimumPoint.Z() << ")"
                    << " 正交性";
                if (it.box.orthoWarn)
                {
                    std::cout << "异常(dotXY=" << it.box.dotXY
                        << ",dotXZ=" << it.box.dotXZ
                        << ",dotYZ=" << it.box.dotYZ << ")";
                    ++boxAssertWarn;
                }
                else
                {
                    std::cout << "良好";
                    ++boxAssertPass;
                }
                std::cout << std::endl;
                solid = BuildBoxSolid(it.box, i);
            }

            // 拓扑合法性 + 实际包围盒
            BRepCheck_Analyzer checker(solid);
            Bnd_Box box;
            BRepBndLib::Add(solid, box);
            double xmin, ymin, zmin, xmax, ymax, zmax;
            box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
            std::cout << "  状态：" << (checker.IsValid() ? "成功(拓扑合法)" : "成功(拓扑异常)")
                << " bbox X[" << xmin << "," << xmax << "] Y[" << ymin << "," << ymax
                << "] Z[" << zmin << "," << zmax << "]" << std::endl;

            solids[i] = solid;
        }
        catch (const Standard_ConstructionError& e)
        {
            std::cout << "  异常：OCCT构造错误：" << e.GetMessageString() << std::endl;
            solid = TopoDS_Shape();
        }
        catch (const std::exception& e)
        {
            std::cout << "  异常：" << e.what() << std::endl;
            solid = TopoDS_Shape();
        }
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (size_t i = 0; i < solids.size(); i++)
    {
        builder.Add(compound, solids[i]);
    }

    TopoDS_Shape result = compound;
    std::vector<TopoDS_Shape> shapes;
    shapes.push_back(result);

    const gp_Dir viewDir(1.0, -2.0, 1.0);
    const gp_Dir viewUp(0.0, 0.0, 1.0);
    std::vector<ProjectedCurve2D> analyticCurves;   // 解析流程临时输出
    std::vector<ProjectedPolyline2D> analyticSupp; // 解析流程离散补充
    ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, analyticCurves);
    ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, analyticSupp);

    // 5. 保存 DXF（解析曲线 + 折线 + 脊线，与 JsonWireSweepTest 同一写出格式）
    const char* dxfPath = "wire_test_Box_Dxf.dxf";
    if (!WriteAxonometricCurvesToDXF(analyticCurves, analyticSupp, dxfPath,NULL))
    {
        std::cout << "错误：JSON 线框模型投影写入 DXF 失败: " << dxfPath << std::endl;
        return -1;
    }
    std::cout << "DXF 已保存: " << dxfPath << std::endl;
    return 0;
}
// ============================================================
// Shape 类定义测试（Shape.h：CShape_Box / CShape_Prism / CShape_Sweep）
// 链路：构造任意方向 shape 实例 -> 经 Get 接口取数 -> 组装
//       JsonBoxItem / JsonPrismItem / JsonSweepItem -> 调用既有建模函数
//       BuildBoxSolid / BuildPrismSolid / BuildSolidFromJsonItem 生成实体
//       -> BRepCheck_Analyzer 拓扑校验 + 体积/顶点断言。
// 覆盖：任意方向 Box、倾斜面 Prism、任意方向折线路径 Sweep（含截面偏移）。
// 返回 0 = 全部通过；-1 = 存在失败项。
// ============================================================
int ShapeClassTest()
{
    std::cout << "\n========== Shape 类定义测试（任意方向 Box/Prism/Sweep） ==========" << std::endl;
    int passCount = 0;
    const int totalCount = 3;
    std::vector<TopoDS_Shape> shapes;   // 成功生成的实体，用于轴测投影

    // ---------------- 1. 任意方向 Box ----------------
    {
        std::cout << "\n[Box] 任意方向包围盒" << std::endl;
        // 任意正交架：Z = normalize(1,2,3)，X = Z×+Z，Y = Z×X（右手系 X×Y=Z）
        gp_Dir boxZ(gp_Vec(1.0, 2.0, 3.0));
        gp_Vec zx = gp_Vec(boxZ).Crossed(gp_Vec(0.0, 0.0, 1.0));
        gp_Dir boxX(zx);
        gp_Dir boxY(gp_Vec(boxZ).Crossed(gp_Vec(boxX)));

        // shape 定义：中心 + 三个半长方向向量（向量长度 = 半尺寸 30/20/10）
        CShape_Box box(gp_Pnt(100.0, 50.0, -40.0),
                       gp_Vec(boxX) * 30.0, gp_Vec(boxY) * 20.0, gp_Vec(boxZ) * 10.0);
        box.SetName("Box_Arbitrary");

        // 经 Get 接口取数
        gp_Pnt minPt = box.GetMinPt();
        gp_Vec dx, dy, dz; box.GetDir(dx, dy, dz);      // 归一化方向
        double sx, sy, sz; box.GetSize(sx, sy, sz);     // 全尺寸 = 2×半长
        std::string dbg; box.GetBoxInfo_Debug(dbg);
        std::cout << dbg;
        std::cout << "  MinPt[" << minPt.X() << ", " << minPt.Y() << ", " << minPt.Z() << "]"
                  << "  Size " << sx << " x " << sy << " x " << sz << std::endl;

        bool dataOk = box.isValid()
            && std::fabs(sx - 60.0) < 1e-9 && std::fabs(sy - 40.0) < 1e-9 && std::fabs(sz - 20.0) < 1e-9
            && std::fabs(dx.Dot(dy)) < 1e-12 && std::fabs(dx.Dot(dz)) < 1e-12 && std::fabs(dy.Dot(dz)) < 1e-12;

        // 组装 JsonBoxItem 并建模
        JsonBoxItem item{};
        item.boxDirX = gp_Dir(dx); item.boxDirY = gp_Dir(dy); item.boxDirZ = gp_Dir(dz);
        item.sizeX = sx; item.sizeY = sy; item.sizeZ = sz;
        item.minimumPoint = minPt;
        item.dotXY = dx.Dot(dy); item.dotXZ = dx.Dot(dz); item.dotYZ = dy.Dot(dz);
        item.orthoWarn = false;

        TopoDS_Shape solid = BuildBoxSolid(item, 1);
        double vol = 0.0;
        if (!solid.IsNull())
        {
            GProp_GProps props; BRepGProp::VolumeProperties(solid, props);
            vol = props.Mass();
        }
        const double expectVol = sx * sy * sz;
        bool topoOk = !solid.IsNull() && BRepCheck_Analyzer(solid).IsValid();
        bool volOk = std::fabs(vol - expectVol) < 1e-6 * expectVol;

        // shape 的 8 顶点应全部落在实体包围盒内
        bool vertsOk = !solid.IsNull();
        if (!solid.IsNull())
        {
            Bnd_Box bbox; BRepBndLib::Add(solid, bbox); bbox.Enlarge(1e-6);
            std::vector<gp_Pnt> verts; box.GetBoxVertices(verts);
            for (const auto& v : verts)
                if (bbox.IsOut(v)) { vertsOk = false; break; }
        }
        std::cout << "  数据回读: " << (dataOk ? "通过" : "失败")
                  << "  拓扑校验: " << (topoOk ? "通过" : "失败")
                  << "  体积: " << vol << " (期望 " << expectVol << ") " << (volOk ? "通过" : "失败")
                  << "  8顶点包围: " << (vertsOk ? "通过" : "失败") << std::endl;
        if (dataOk && topoOk && volOk && vertsOk) { ++passCount; std::cout << "  [Box] 通过" << std::endl; }
        else std::cout << "  [Box] 失败" << std::endl;
        if (!solid.IsNull()) shapes.push_back(solid);
    }

    // ---------------- 2. 倾斜面 Prism ----------------
    {
        std::cout << "\n[Prism] 倾斜面棱柱" << std::endl;
        gp_Dir prismN(gp_Vec(1.0, -2.0, 2.0));   // 任意倾斜法向
        gp_Vec px = gp_Vec(prismN).Crossed(gp_Vec(0.0, 0.0, 1.0));
        gp_Dir prismX(px);
        gp_Dir prismY(gp_Vec(prismN).Crossed(gp_Vec(prismX)));
        const gp_Pnt prismOrg(10.0, 20.0, 30.0);

        // 面内五边形轮廓（局部 UV，首末点重合闭合），以 3D 点形式存入 shape
        const double uv[][2] = { {0,0},{40,0},{45,10},{20,25},{0,15},{0,0} };
        const size_t uvCount = sizeof(uv) / sizeof(uv[0]);
        std::vector<gp_Pnt> loopPts;
        for (size_t i = 0; i < uvCount; ++i)
            loopPts.push_back(prismOrg.Translated(gp_Vec(prismX) * uv[i][0] + gp_Vec(prismY) * uv[i][1]));
        double area = 0.0;  // 鞋带公式求面内多边形面积
        for (size_t i = 0; i + 1 < uvCount; ++i)
            area += uv[i][0] * uv[i + 1][1] - uv[i + 1][0] * uv[i][1];
        area = std::fabs(area) * 0.5;

        const double thick = 12.0;
        CShape_Prism prismShape(CGeCurveColl(loopPts),
                                gp_Vec(prismX), gp_Vec(prismY), gp_Vec(prismN), thick);
        prismShape.SetName("Prism_Inclined");

        // 经 Get 接口取数
        CGeCurveColl outLoop; prismShape.GetGeColl(outLoop);
        gp_Vec pdx, pdy, pdz; prismShape.GetDir(pdx, pdy, pdz);  // 归一化三方向
        double outThick = prismShape.GetThick();
        bool dataOk = outLoop.Points().size() == loopPts.size()
            && std::fabs(outThick - thick) < 1e-12
            && std::fabs(pdz.Dot(gp_Vec(prismN)) - 1.0) < 1e-12;

        JsonPrismItem item{};
        item.outlinePts = outLoop.Points();
        item.planeNormal = gp_Dir(pdz);
        item.thickness = outThick;

        TopoDS_Shape solid = BuildPrismSolid(item, 1);
        double vol = 0.0;
        if (!solid.IsNull())
        {
            GProp_GProps props; BRepGProp::VolumeProperties(solid, props);
            vol = props.Mass();
        }
        const double expectVol = area * outThick;
        bool topoOk = !solid.IsNull() && BRepCheck_Analyzer(solid).IsValid();
        bool volOk = std::fabs(vol - expectVol) < 1e-6 * expectVol;
        std::cout << "  轮廓点数: " << outLoop.Points().size() << "  厚度: " << outThick
                  << "  法向[" << pdz.X() << ", " << pdz.Y() << ", " << pdz.Z() << "]" << std::endl;
        std::cout << "  数据回读: " << (dataOk ? "通过" : "失败")
                  << "  拓扑校验: " << (topoOk ? "通过" : "失败")
                  << "  体积: " << vol << " (期望 " << expectVol << ") " << (volOk ? "通过" : "失败") << std::endl;
        if (dataOk && topoOk && volOk) { ++passCount; std::cout << "  [Prism] 通过" << std::endl; }
        else std::cout << "  [Prism] 失败" << std::endl;
        if (!solid.IsNull()) shapes.push_back(solid);
    }

    // ---------------- 3. 任意方向 Sweep（含截面沿路径偏移） ----------------
    {
        std::cout << "\n[Sweep] 任意方向扫掠" << std::endl;
        const gp_Pnt pathStart(0.0, 0.0, 0.0);
        const gp_Pnt pathEnd(120.0, 40.0, 60.0);   // 任意方向路径（非轴对齐）
        gp_Dir tangent(gp_Vec(pathStart, pathEnd));

        // 扫掠平面法向：取垂直于切向的任意法向
        gp_Vec nsV = gp_Vec(0.0, 0.0, 1.0).Crossed(gp_Vec(tangent));
        if (nsV.Magnitude() < 1e-9) nsV = gp_Vec(1.0, 0.0, 0.0).Crossed(gp_Vec(tangent));
        gp_Dir sweepPlaneN(nsV);

        // 截面框架与 BuildSolidFromJsonItem 一致：U = n_sweep × T，V = T × U
        gp_Vec U = gp_Vec(sweepPlaneN).Crossed(gp_Vec(tangent));
        gp_Vec V = gp_Vec(tangent).Crossed(U);

        // 截面（局部 UV 矩形，首末点重合闭合），以 3D 点形式存入 shape
        const double uv[][2] = { {0,0},{10,0},{10,6},{0,6},{0,0} };
        std::vector<gp_Pnt> loop3d;
        for (const auto& q : uv)
            loop3d.push_back(pathStart.Translated(U * q[0] + V * q[1]));
        const double secArea = 10.0 * 6.0;

        // shape 定义：轨迹 + 3D 截面轮廓 + 截面沿路径偏移 15
        CShape_Sweep sweepShape(CGeCurveColl(std::vector<gp_Pnt>{pathStart, pathEnd}),
                                CGeCurveColl(loop3d), 15.0);
        sweepShape.SetSweepDir(gp_Vec(sweepPlaneN));
        sweepShape.SetSeciontDir(gp_Vec(tangent));
        sweepShape.SetSEAng(25.0, 35.0);

        // 接口回读验证（切角仅验证存取；建模前重置为 0 使体积 = 截面积 × 长度）
        double staAng = 0.0, endAng = 0.0;
        sweepShape.GetSEAng(staAng, endAng);
        bool angOk = std::fabs(staAng - 25.0) < 1e-12 && std::fabs(endAng - 35.0) < 1e-12;
        sweepShape.SetSEAng(0.0, 0.0);
        sweepShape.GetSEAng(staAng, endAng);

        CGeCurveColl pathColl, loopColl;
        sweepShape.GetInfo(pathColl, loopColl);
        const double offset = sweepShape.GetOffset();
        gp_Vec nSweep = sweepShape.GetSweepDir().Normalized();
        gp_Vec secDir = sweepShape.GetSectionDir().Normalized();

        const std::vector<gp_Pnt>& pathPts = pathColl.Points();
        const gp_Pnt pStart = pathPts.front(), pEnd = pathPts.back();
        gp_Dir T(gp_Vec(pStart, pEnd));
        const gp_Pnt anchor = pStart.Translated(gp_Vec(T) * offset); // 截面偏移后的锚点

        // 3D 截面轮廓 -> 截面局部 UV（框架同 BuildSolidFromJsonItem）
        gp_Vec U2 = nSweep.Crossed(gp_Vec(T));
        gp_Vec V2 = gp_Vec(T).Crossed(U2);
        std::vector<gp_Pnt2d> sectionPts;
        for (const gp_Pnt& p : loopColl.Points())
        {
            gp_Vec r(anchor, p);
            sectionPts.push_back(gp_Pnt2d(r.Dot(U2), r.Dot(V2)));
        }

        bool dataOk = angOk
            && std::fabs(offset - 15.0) < 1e-12
            && pathPts.size() == 2 && loopColl.Points().size() == 5
            && std::fabs(nSweep.Dot(gp_Vec(sweepPlaneN)) - 1.0) < 1e-12
            && std::fabs(secDir.Dot(gp_Vec(tangent)) - 1.0) < 1e-12;

        JsonSweepItem item{};
        item.sectionPts = sectionPts;
        item.sectionNormal = gp_Dir(secDir);
        item.sweepStart = anchor;
        item.sweepEnd = pEnd;
        item.sweepPlaneNormal = gp_Dir(nSweep);
        item.startAng = staAng;
        item.endAng = endAng;

        TopoDS_Shape solid = BuildSolidFromJsonItem(item, 1);
        double vol = 0.0;
        if (!solid.IsNull())
        {
            GProp_GProps props; BRepGProp::VolumeProperties(solid, props);
            vol = props.Mass();
        }
        const double pathLen = pStart.Distance(pEnd) - offset;
        const double expectVol = secArea * pathLen;
        bool topoOk = !solid.IsNull() && BRepCheck_Analyzer(solid).IsValid();
        bool volOk = std::fabs(vol - expectVol) < 1e-3 * expectVol;
        std::cout << "  路径: (" << pStart.X() << "," << pStart.Y() << "," << pStart.Z() << ") -> ("
                  << pEnd.X() << "," << pEnd.Y() << "," << pEnd.Z() << ")  偏移: " << offset
                  << "  有效长度: " << pathLen << std::endl;
        std::cout << "  数据回读: " << (dataOk ? "通过" : "失败")
                  << "  拓扑校验: " << (topoOk ? "通过" : "失败")
                  << "  体积: " << vol << " (期望 " << expectVol << ") " << (volOk ? "通过" : "失败") << std::endl;
        if (dataOk && topoOk && volOk) { ++passCount; std::cout << "  [Sweep] 通过" << std::endl; }
        else std::cout << "  [Sweep] 失败" << std::endl;
        if (!solid.IsNull()) shapes.push_back(solid);
    }

    std::cout << "\nShape 类定义测试汇总: " << passCount << "/" << totalCount
              << (passCount == totalCount ? " 全部通过" : " 存在失败") << std::endl;

    // ---------------- 4. 轴测投影并保存 DXF ----------------
    // 与 JsonWireTest2 同一视图变换与写出格式：解析曲线 + 离散补充折线
    if (!shapes.empty())
    {
        const gp_Dir viewDir(1.0, -2.0, 1.0);
        const gp_Dir viewUp(0.0, 0.0, 1.0);
        std::vector<ProjectedCurve2D> analyticCurves;    // 解析流程输出（线段/圆弧/圆）
        std::vector<ProjectedPolyline2D> analyticSupp;   // 解析流程离散补充
        ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, analyticCurves);
        ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, analyticSupp);

        const char* dxfPath = "shape_class_test.dxf";
        if (!WriteAxonometricCurvesToDXF(analyticCurves, analyticSupp, dxfPath, NULL))
        {
            std::cout << "错误：Shape 类定义模型投影写入 DXF 失败: " << dxfPath << std::endl;
            return -1;
        }
        std::cout << "DXF 已保存: " << dxfPath
                  << "（实体 " << shapes.size() << " 个，解析曲线 " << analyticCurves.size()
                  << " 条，离散补充 " << analyticSupp.size() << " 条）" << std::endl;
    }

    return passCount == totalCount ? 0 : -1;
}

// ============================================================
// CShape_Box::CombineBox 合并测试
// 覆盖：1) 两盒远离  2) 两盒相交（B 轴序置换）  3) 两盒包含
//       4) 任意方向同向相交  5) B 绕 Z 旋转 45°（合并 = A 轴系 AABB）
//       6) 轴无法配对的退化盒（应正确拒绝，返回 false）
// 每例校验：合并后中心/三半长向量与期望一致（isSame，容差 1e-6），
//           且合并盒在自身轴系下包围两原始盒的全部 8 顶点。
// 几何输出：每例 原盒A + 原盒B + 合并盒 -> 轴测投影 DXF。
// 返回 0 = 全部通过；-1 = 存在失败项。
// ============================================================
int CombineBoxTest()
{
    std::cout << "\n========== CShape_Box::CombineBox 合并测试 ==========" << std::endl;
    int passCount = 0;
    const int totalCount = 6;
    std::vector<TopoDS_Shape> shapes;   // 原盒 + 合并盒，用于轴测投影
    size_t solidIdx = 0;

    // CShape_Box -> JsonBoxItem -> 拉伸实体（供 DXF 输出）
    auto buildSolid = [&](const CShape_Box& b) -> TopoDS_Shape
    {
        JsonBoxItem item{};
        gp_Vec dx, dy, dz; b.GetDir(dx, dy, dz);
        double sx, sy, sz; b.GetSize(sx, sy, sz);
        item.boxDirX = gp_Dir(dx); item.boxDirY = gp_Dir(dy); item.boxDirZ = gp_Dir(dz);
        item.sizeX = sx; item.sizeY = sy; item.sizeZ = sz;
        item.minimumPoint = b.GetMinPt();
        item.dotXY = dx.Dot(dy); item.dotXZ = dx.Dot(dz); item.dotYZ = dy.Dot(dz);
        item.orthoWarn = false;
        return BuildBoxSolid(item, ++solidIdx);
    };

    // 合并盒在自身轴系下包围两原始盒的全部顶点
    auto boundsOk = [](const CShape_Box& merged, const CShape_Box& a, const CShape_Box& b) -> bool
    {
        const double tol = 1e-6;
        gp_Vec ax, ay, az; merged.GetDir(ax, ay, az);
        double hx, hy, hz; merged.GetSize(hx, hy, hz);
        hx *= 0.5; hy *= 0.5; hz *= 0.5;
        const gp_Pnt c = merged.GetCenter();
        std::vector<gp_Pnt> va, vb;
        a.GetBoxVertices(va); b.GetBoxVertices(vb);
        for (int pass = 0; pass < 2; ++pass)
        {
            const std::vector<gp_Pnt>& vs = (pass == 0 ? va : vb);
            for (const gp_Pnt& p : vs)
            {
                gp_Vec r(c, p);
                if (std::fabs(r.Dot(ax)) > hx + tol) return false;
                if (std::fabs(r.Dot(ay)) > hy + tol) return false;
                if (std::fabs(r.Dot(az)) > hz + tol) return false;
            }
        }
        return true;
    };

    // 单例执行：合并 -> isSame 期望比对 -> 包围校验 -> 实体输出
    auto runCase = [&](const char* name, const CShape_Box& a, const CShape_Box& b,
                       const gp_Pnt& expC, const gp_Vec& expHx,
                       const gp_Vec& expHy, const gp_Vec& expHz) -> bool
    {
        CShape_Box merged = a;
        bool ok = merged.CombineBox(b);
        CShape_Box expected(expC, expHx, expHy, expHz);
        bool sameOk = ok && merged.isSame(expected);
        bool bndOk = ok && boundsOk(merged, a, b);
        double msx, msy, msz; merged.GetSize(msx, msy, msz);
        gp_Pnt mc = merged.GetCenter();
        std::cout << "[" << name << "] 合并: " << (ok ? "成功" : "失败")
                  << "  中心(" << mc.X() << ", " << mc.Y() << ", " << mc.Z() << ")"
                  << "  尺寸 " << msx << " x " << msy << " x " << msz
                  << "  期望一致: " << (sameOk ? "通过" : "失败")
                  << "  包围校验: " << (bndOk ? "通过" : "失败") << std::endl;
        if (ok)
        {
            TopoDS_Shape sa = buildSolid(a);
            TopoDS_Shape sb = buildSolid(b);
            TopoDS_Shape sm = buildSolid(merged);
            if (!sa.IsNull()) shapes.push_back(sa);
            if (!sb.IsNull()) shapes.push_back(sb);
            if (!sm.IsNull()) shapes.push_back(sm);
        }
        return ok && sameOk && bndOk;
    };

    const gp_Vec X(1.0, 0.0, 0.0), Y(0.0, 1.0, 0.0), Z(0.0, 0.0, 1.0);

    // ---- Case 1：两盒远离（轴对齐，沿 X 拉开） ----
    // A: X∈[-10,10]，B: X∈[90,110] -> 合并 X∈[-10,110]
    if (runCase("Case1 远离",
        CShape_Box(gp_Pnt(0.0, 0.0, 0.0),   X * 10.0, Y * 10.0, Z * 10.0),
        CShape_Box(gp_Pnt(100.0, 0.0, 0.0), X * 10.0, Y * 10.0, Z * 10.0),
        gp_Pnt(50.0, 0.0, 0.0), X * 60.0, Y * 10.0, Z * 10.0)) ++passCount;

    // ---- Case 2：两盒相交（B 轴序置换：dirX=Ŷ·10, dirY=Ẑ·10, dirZ=X̂·20） ----
    // A: X∈[-20,20] Y∈[140,160]；B: X∈[-5,35] Y∈[145,165] -> 合并 X∈[-20,35] Y∈[140,165]
    if (runCase("Case2 相交(轴置换)",
        CShape_Box(gp_Pnt(0.0, 150.0, 0.0), X * 20.0, Y * 10.0, Z * 10.0),
        CShape_Box(gp_Pnt(15.0, 155.0, 0.0), Y * 10.0, Z * 10.0, X * 20.0),
        gp_Pnt(7.5, 152.5, 0.0), X * 27.5, Y * 12.5, Z * 10.0)) ++passCount;

    // ---- Case 3：两盒包含（B 完全在 A 内，合并 = A） ----
    if (runCase("Case3 包含",
        CShape_Box(gp_Pnt(0.0, 300.0, 0.0), X * 30.0, Y * 20.0, Z * 10.0),
        CShape_Box(gp_Pnt(5.0, 305.0, 2.0), X * 10.0, Y * 5.0,  Z * 5.0),
        gp_Pnt(0.0, 300.0, 0.0), X * 30.0, Y * 20.0, Z * 10.0)) ++passCount;

    // ---- Case 4：任意方向同向相交（旋转架 Z=normalize(1,2,3)，B 沿架 X 偏移 25） ----
    {
        gp_Dir fz(gp_Vec(1.0, 2.0, 3.0));
        gp_Vec fxv = gp_Vec(fz).Crossed(gp_Vec(0.0, 0.0, 1.0));
        gp_Dir fx(fxv);
        gp_Dir fy(gp_Vec(fz).Crossed(gp_Vec(fx)));
        const gp_Pnt ca(10.0, 470.0, 30.0);
        const gp_Pnt cb = ca.Translated(gp_Vec(fx) * 25.0);
        // 架内 A: X∈[-15,15]，B: X∈[10,40] -> 合并 X∈[-15,40]（中心偏移 12.5，半长 27.5）
        if (runCase("Case4 任意方向相交",
            CShape_Box(ca, gp_Vec(fx) * 15.0, gp_Vec(fy) * 10.0, gp_Vec(fz) * 5.0),
            CShape_Box(cb, gp_Vec(fx) * 15.0, gp_Vec(fy) * 10.0, gp_Vec(fz) * 5.0),
            ca.Translated(gp_Vec(fx) * 12.5),
            gp_Vec(fx) * 27.5, gp_Vec(fy) * 10.0, gp_Vec(fz) * 5.0)) ++passCount;
    }

    // ---- Case 5：B 绕 Z 旋转 45°（合并 = A 轴系下的 AABB） ----
    {
        const double c = std::sqrt(0.5);   // cos45° = sin45°
        gp_Vec bx(c, c, 0.0), by(-c, c, 0.0);
        const double h = 10.0 * c + 5.0 * c;   // 旋转 B 在 X/Y 向的半宽 ≈ 10.6066
        // X: [min(-10, 50-h), max(10, 50+h)] = [-10, 50+h]；Y: [-h, h]；Z: [-10, 10]
        if (runCase("Case5 B旋转45度",
            CShape_Box(gp_Pnt(0.0, 600.0, 0.0),  X * 10.0, Y * 10.0, Z * 10.0),
            CShape_Box(gp_Pnt(50.0, 600.0, 0.0), bx * 10.0, by * 5.0, Z * 5.0),
            gp_Pnt(20.0 + h * 0.5, 600.0, 0.0), X * (30.0 + h * 0.5), Y * h, Z * 10.0)) ++passCount;
    }

    // ---- Case 6：轴无法配对的退化盒（dirX 与 dirY 同向，应返回 false） ----
    {
        CShape_Box a(gp_Pnt(0.0, 750.0, 0.0),  X * 10.0, Y * 10.0, Z * 10.0);
        CShape_Box b(gp_Pnt(50.0, 750.0, 0.0), X * 10.0, X * 5.0,  Z * 5.0);   // 退化：两轴同向
        CShape_Box merged = a;
        bool ok = merged.CombineBox(b);
        std::cout << "[Case6 退化盒] 合并返回: " << (ok ? "true" : "false")
                  << (ok ? "  失败（应拒绝）" : "  通过（正确拒绝）") << std::endl;
        if (!ok) ++passCount;
    }

    std::cout << "\nCombineBox 测试汇总: " << passCount << "/" << totalCount
              << (passCount == totalCount ? " 全部通过" : " 存在失败") << std::endl;

    // ---- 轴测投影并保存 DXF（与既有测试同一视图变换与写出格式） ----
    if (!shapes.empty())
    {
        const gp_Dir viewDir(1.0, -2.0, 1.0);
        const gp_Dir viewUp(0.0, 0.0, 1.0);
        std::vector<ProjectedCurve2D> analyticCurves;
        std::vector<ProjectedPolyline2D> analyticSupp;
        ProjectCurveSetTo2DCurves(shapes, viewDir, viewUp, analyticCurves);
        ProjectBoxSetTo2DCurves(shapes, viewDir, viewUp, analyticSupp);

        const char* dxfPath = "combine_box_test.dxf";
        if (!WriteAxonometricCurvesToDXF(analyticCurves, analyticSupp, dxfPath, NULL))
        {
            std::cout << "错误：CombineBox 测试模型投影写入 DXF 失败: " << dxfPath << std::endl;
            return -1;
        }
        std::cout << "DXF 已保存: " << dxfPath
                  << "（实体 " << shapes.size() << " 个，解析曲线 " << analyticCurves.size()
                  << " 条，离散补充 " << analyticSupp.size() << " 条）" << std::endl;
    }

    return passCount == totalCount ? 0 : -1;
}
