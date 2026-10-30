#include "Tests.h"
#include "Viewer.h"

int main()
{
    OCCT_Test();

    // 带孔棱柱测试：BuildFaceWithHoles 生成带孔面 + MakePrism 拉伸成棱柱
    BuildHollowPrismTest();

    // 倾斜平面真实数据测试
    BuildFaceOnInclinedPlaneTest();

    // L型扫掠 DXF 测试（L型脊线 + 真实截面，详见 LineSweepDXFTest）
    LineSweepDXFTest();

    // 乱序轨迹段扫掠 DXF 测试（3段乱序Segment重排为U型开口轮廓后扫掠）
    UnorderedPathSweepDXFTest();

    // 收集各测试生成的模型，用于视图显示
    std::vector<TopoDS_Shape> viewShapes;

    // 收集 JSON 包围盒中 PlaneInfo 对应的红色交叉线
    std::vector<TopoDS_Shape> boxCrossShapes;

    // 收集 JSON 包围盒的编号文字标注（按 JSON 顺序从 1 开始，位于各 box 自身 Y 正方向面中心）
    std::vector<TextViewLabel> boxNumberLabels;

    // 收集同一 GroupID 的 box 合并后的总 box（透明实体 + 虚线边框显示）
    std::vector<TopoDS_Shape> boxGroupShapes;

    // JSON 线框扫掠测试（test3.txt：闭合多段线截面 + 单段路径，逐条建模汇总）
    if(0)
    {
        TopoDS_Shape jsonSweepModel;
        JsonWireSweepTest("test3.txt", jsonSweepModel);
        if (!jsonSweepModel.IsNull())
        {
            viewShapes.push_back(jsonSweepModel);
        }
    }

    // JSON 线框综合测试（test4.txt：Prism 棱柱 + Sweep 扫掠混合条目，逐条建模汇总）
    // 屏蔽，不用测试
    //{
    //    TopoDS_Shape jsonWireModel;
    //    JsonWireTest("test4.txt", jsonWireModel);
    //}
    if(0)
    {
        TopoDS_Shape jsonWireModel;
        JsonWireTest2("test4.txt", jsonWireModel);
        if (!jsonWireModel.IsNull())
        {
           // viewShapes.push_back(jsonWireModel);
        }
    }

    // JSON 包围盒测试（test6.txt：数组内 {"Box":{...}}，逐条生成 OCCT box 实体并汇总；
    // 带 PlaneInfo 的 Box 额外输出对应面上的交叉线，并输出按 JSON 顺序从 1 开始的编号标注）
    {
        TopoDS_Shape jsonBoxModel;
        JsonBoxTest("test6.txt", jsonBoxModel, boxCrossShapes, boxNumberLabels, boxGroupShapes);
        if (!jsonBoxModel.IsNull())
        {
            viewShapes.push_back(jsonBoxModel);
        }
    }

    // Shape 类定义测试（Shape.h：任意方向 Box/Prism/Sweep 经 Get 接口取数建模）
    ShapeClassTest();

    // CShape_Box::CombineBox 合并测试（远离/相交/包含/任意方向，输出 DXF）
    CombineBoxTest();

    // ========================================================================
    // 可视化：将上面测试生成的模型显示到 OCCT 视图
    // 交互：左键旋转 / 右键(中键)平移 / 滚轮缩放；F 全图，R 重置，
    //       S 着色，W 线框，1/2/3/4 标准视图，Esc 退出
    // ========================================================================
    if (!viewShapes.empty())
    {
        COCCTViewer viewer;
        if (viewer.Create(1280, 800, "OCCT 视图 - 测试模型"))
        {
            viewer.DisplayShapes(viewShapes);
            // 包围盒平面交叉线：红色线框叠加显示
            for (size_t i = 0; i < boxCrossShapes.size(); ++i)
            {
                viewer.DisplayShape(boxCrossShapes[i],
                                    Quantity_Color(Quantity_NOC_RED), AIS_WireFrame);
            }
            // 包围盒编号：红色文字标注在各自 box 自身 Y 正方向面的中心（按 JSON 顺序从 1 编号）
            for (size_t i = 0; i < boxNumberLabels.size(); ++i)
            {
                viewer.DisplayText(boxNumberLabels[i].position,
                                   TCollection_AsciiString(boxNumberLabels[i].text.c_str()),
                                   Quantity_Color(Quantity_NOC_RED));
            }
            // 同一 GroupID 的总 box：透明实体 + 绿色虚线边框，仅可见边框
            for (size_t i = 0; i < boxGroupShapes.size(); ++i)
            {
                viewer.DisplayShapeDashed(boxGroupShapes[i],
                                          Quantity_Color(Quantity_NOC_GREEN), 1.0, 2.0);
            }
            viewer.FitAll();
            viewer.Run();
        }
    }

    // 多方向正交投影：视线方向 + 屏幕上方向作为参数，4 个视图在同一 DXF 中左右排开
    //std::vector<ProjectedPolyline2D> Curves;
   // OCCT_ProjectTest(gp_Dir(1,0,0), gp_Dir(0,0,1),   0, Curves);   // 右视（相机在 +X 侧）
    //OCCT_ProjectTest(gp_Dir(0,1,0), gp_Dir(0,0,1), 200, Curves);   // 前视（相机在 +Y 侧）
    //OCCT_ProjectTest(gp_Dir(0,0,1), gp_Dir(1,0,0), 400, Curves);   // 俯视（相机在 +Z 侧，屏幕上=world+X）
    //OCCT_ProjectTest(gp_Dir(1,1,1), gp_Dir(1,0,0), 600, Curves);   // 等轴测

    //OcclusionDemo();   // 遮挡消隐能力验证（结果不写入 DXF）

    // 将投影轮廓写入 DXF 文件
    //if (!WriteProjectedCurvesToDXF(Curves, "projection.dxf"))
    //{
	    //std::cout << "投影轮廓写入 DXF 失败！" << std::endl;
	   // return -1;
    //}

    //std::cout << "\n========== dxflib DLL 导入测试 ==========" << std::endl;
    //DXFLIB_Test();
    return 0;
}