#pragma once

// ============================================================================
// Tests.h —— 测试函数入口声明 + 测试用自定义数据结构
// 全部测试函数实现统一收集在 Tests.cpp；main.cpp 仅包含本头文件并逐个调用。
// 测试中使用的自定义数据结构（结构体 / 枚举 / 辅助类）统一在本头文件中定义。
// ============================================================================

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir.hxx>
#include <iostream>
#include <streambuf>
#include <string>
#include <vector>

#include "dl_creationadapter.h"

// 视图文字标注（3D 锚点 + 文本内容），由 main.cpp 通过 COCCTViewer 显示
struct TextViewLabel
{
    gp_Pnt position;   // 标注锚点（模型坐标）
    std::string text;  // 标注文字（如编号 "1"）
};

// ---------- 扫掠轨迹 / 截面轮廓数据 ----------

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

// ---------- JSON 线框数据 ----------

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

    // 可选平面信息（Box.PlaneInfo）：PlaneDir 为平面法向，PlanePos 为平面上一点。
    // 有该信息时，JsonBoxTest 会在对应面上绘制红色交叉线。
    bool   hasPlane;
    gp_Dir planeDir;
    gp_Pnt planePos;
    int    groupID; //组ID
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

// ---------- 测试辅助类 ----------

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

// ---------- 测试函数入口 ----------

int OCCT_Test();
int BuildHollowPrismTest();
int BuildFaceOnInclinedPlaneTest();
int LineSweepDXFTest();
int UnorderedPathSweepDXFTest();
int JsonWireSweepTest(const char* file, TopoDS_Shape& resultModel);
int JsonWireTest2(const char* file, TopoDS_Shape& resultModel);
int JsonBoxTest(const char* file, TopoDS_Shape& resultModel,
                std::vector<TopoDS_Shape>& crossShapes,
                std::vector<TextViewLabel>& numberLabels,
                std::vector<TopoDS_Shape>& groupBoxes);
int ShapeClassTest();
int CombineBoxTest();
