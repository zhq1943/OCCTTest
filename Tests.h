#pragma once

// ============================================================================
// Tests.h —— 测试函数入口声明
// 全部测试函数实现统一收集在 Tests.cpp；main.cpp 仅包含本头文件并逐个调用。
// ============================================================================

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <string>
#include <vector>

// 视图文字标注（3D 锚点 + 文本内容），由 main.cpp 通过 COCCTViewer 显示
struct TextViewLabel
{
    gp_Pnt position;   // 标注锚点（模型坐标）
    std::string text;  // 标注文字（如编号 "1"）
};

int OCCT_Test();
int BuildHollowPrismTest();
int BuildFaceOnInclinedPlaneTest();
int LineSweepDXFTest();
int UnorderedPathSweepDXFTest();
int JsonWireSweepTest(const char* file, TopoDS_Shape& resultModel);
int JsonWireTest2(const char* file, TopoDS_Shape& resultModel);
int JsonBoxTest(const char* file, TopoDS_Shape& resultModel,
                std::vector<TopoDS_Shape>& crossShapes,
                std::vector<TextViewLabel>& numberLabels);
int ShapeClassTest();
int CombineBoxTest();
