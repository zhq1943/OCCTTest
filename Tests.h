#pragma once

// ============================================================================
// Tests.h —— 测试函数入口声明
// 全部测试函数实现统一收集在 Tests.cpp；main.cpp 仅包含本头文件并逐个调用。
// ============================================================================

#include <TopoDS_Shape.hxx>

int OCCT_Test();
int BuildHollowPrismTest();
int BuildFaceOnInclinedPlaneTest();
int LineSweepDXFTest();
int UnorderedPathSweepDXFTest();
int JsonWireSweepTest(const char* file, TopoDS_Shape& resultModel);
int JsonWireTest2(const char* file, TopoDS_Shape& resultModel);
int ShapeClassTest();
int CombineBoxTest();
