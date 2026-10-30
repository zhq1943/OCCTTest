#pragma once

// ============================================================================
// Viewer.h —— OCCT 可视化模块（渲染显示）
// COCCTViewer 封装 OCCT 可视化三要素：
//   - V3d_Viewer / V3d_View（视图与相机）
//   - AIS_InteractiveContext（交互对象上下文，负责模型显示）
//   - WNT_Window + Win32 窗口（渲染目标，控制台程序下直接弹窗查看）
// 用法：
//   COCCTViewer viewer;
//   if (viewer.Create(1280, 800, "OCCT 视图"))
//   {
//       viewer.DisplayShape(shape, Quantity_Color(Quantity_NOC_STEELBLUE));
//       viewer.FitAll();
//       viewer.Run();   // 进入消息循环，关闭窗口后返回
//   }
// 交互：左键旋转 / 右键或中键平移 / 滚轮缩放
//       F 全图适配，R 重置视角，S 着色，W 线框，1/2/3/4 标准视图，Esc 退出
// ============================================================================

#include <Windows.h>

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_TextLabel.hxx>
#include <TCollection_AsciiString.hxx>
#include <gp_Pnt.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>

#include <string>
#include <vector>

class COCCTViewer
{
public:
    COCCTViewer();
    ~COCCTViewer();

    // 创建窗口并初始化 OCCT 查看器（驱动 / 查看器 / 视图 / 上下文）
    bool Create(int width = 1280, int height = 800, const char* title = "OCCT Viewer");

    // 显示模型（AIS_Shape），可指定颜色与显示模式（默认着色）
    void DisplayShape(const TopoDS_Shape& shape);
    void DisplayShape(const TopoDS_Shape& shape, const Quantity_Color& color,
                      AIS_DisplayMode mode = AIS_Shaded);
    void DisplayShapes(const std::vector<TopoDS_Shape>& shapes);
    // 透明实体 + 虚线边框：面全透明不可见，仅以虚线绘制面边界
    void DisplayShapeDashed(const TopoDS_Shape& shape, const Quantity_Color& color,
                            double transparency = 1.0, double lineWidth = 2.0);
    // 显示 3D 文字标注（AIS_TextLabel：屏幕朝向、固定像素高度）
    void DisplayText(const gp_Pnt& position, const TCollection_AsciiString& text,
                     const Quantity_Color& color, double height = 22.0);

    void RemoveAll();                                   // 清空已显示对象
    void FitAll();                                      // 全图适配并刷新
    void Redraw();                                      // 刷新视图
    void SetDisplayModeAll(AIS_DisplayMode mode);       // 切换全部对象显示模式
    void SetOrientation(V3d_TypeOfOrientation orient);  // 设置标准视图方向

    // 进入模态消息循环，直到窗口关闭；返回后窗口与视图资源已释放
    int Run();

    bool IsCreated() const { return m_hWnd != NULL; }

    // OCCT 原生对象访问（需要更底层控制时使用）
    Handle(AIS_InteractiveContext) Context() const { return m_context; }
    Handle(V3d_View)               View()    const { return m_view; }
    Handle(V3d_Viewer)             Viewer()  const { return m_viewer; }

private:
    bool createWindow(int width, int height, const char* title);
    bool initOCCT();
    void destroy();

    static LRESULT CALLBACK wndProcThunk(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
    LRESULT onMessage(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

    void onLButtonDown(int x, int y);
    void onLButtonUp(int x, int y);
    void onMouseMove(int x, int y, WPARAM keys);
    void onMouseWheel(short delta);
    void onKeyDown(WPARAM key);
    void onSize();

private:
    HWND m_hWnd;                                 // Win32 窗口句柄

    Handle(Aspect_DisplayConnection) m_display;  // 显示连接（Windows 下为空句柄即可）
    Handle(OpenGl_GraphicDriver)     m_driver;   // OpenGL 图形驱动
    Handle(V3d_Viewer)               m_viewer;   // OCCT 查看器
    Handle(V3d_View)                 m_view;     // 视图（相机）
    Handle(AIS_InteractiveContext)   m_context;  // 交互上下文
    Handle(WNT_Window)               m_ntWindow; // 渲染目标窗口封装

    std::vector<Handle(AIS_Shape)>   m_displayed;// 已显示的模型对象

    bool m_lDown;                                // 左键按下（旋转）
    bool m_rDown;                                // 右键按下（平移）
    bool m_mDown;                                // 中键按下（平移）
    int  m_lastX;                                // 上一次鼠标位置
    int  m_lastY;
};
