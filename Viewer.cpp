#include "Viewer.h"

// ============================================================================
// Viewer.cpp —— OCCT 可视化模块实现
// 基于 OCCT 的 OpenGl 驱动 + V3d 视图 + AIS 交互上下文，渲染目标为
// 本类自行创建的 Win32 窗口（WNT_Window 封装已有 HWND）。
// ============================================================================

#include <Aspect_GradientFillMethod.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_NameOfColor.hxx>
#include <Quantity_TypeOfColor.hxx>
#include <TCollection_ExtendedString.hxx>
#include <V3d_TypeOfOrientation.hxx>

#include <windowsx.h>   // GET_X_LPARAM / GET_Y_LPARAM

namespace
{
    const wchar_t* kWndClassName = L"OCCTViewerWindow";
}

// ----------------------------------------------------------------------------
COCCTViewer::COCCTViewer()
    : m_hWnd(NULL)
    , m_lDown(false)
    , m_rDown(false)
    , m_mDown(false)
    , m_lastX(0)
    , m_lastY(0)
{
}

// ----------------------------------------------------------------------------
COCCTViewer::~COCCTViewer()
{
    destroy();
}

// ----------------------------------------------------------------------------
// 创建窗口并初始化 OCCT 查看器
bool COCCTViewer::Create(int width, int height, const char* title)
{
    if (m_hWnd != NULL)
    {
        return true;    // 已创建
    }
    return createWindow(width, height, title) && initOCCT();
}

// ----------------------------------------------------------------------------
// 注册窗口类（仅一次）并创建 Win32 窗口
bool COCCTViewer::createWindow(int width, int height, const char* title)
{
    HINSTANCE hInst = GetModuleHandleW(NULL);

    static bool classRegistered = false;
    if (!classRegistered)
    {
        WNDCLASSEXW wc    = {};
        wc.cbSize         = sizeof(wc);
        wc.style          = CS_OWNDC | CS_HREDRAW | CS_VREDRAW; // OpenGL 必须 CS_OWNDC
        wc.lpfnWndProc    = &COCCTViewer::wndProcThunk;
        wc.hInstance      = hInst;
        wc.hCursor        = LoadCursorW(NULL, IDC_ARROW);
        wc.hbrBackground  = NULL;   // 背景由 OpenGL 绘制，避免闪烁
        wc.lpszClassName  = kWndClassName;
        if (RegisterClassExW(&wc) == 0)
        {
            return false;
        }
        classRegistered = true;
    }

    // 标题 char -> wchar_t（项目未启用 /utf-8，窄字面量按系统代码页 GBK 编码）
    wchar_t wTitle[256] = {};
    MultiByteToWideChar(CP_ACP, 0, title, -1, wTitle, 255);

    m_hWnd = CreateWindowExW(0, kWndClassName, wTitle,
                             WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
                             CW_USEDEFAULT, CW_USEDEFAULT, width, height,
                             NULL, NULL, hInst, NULL);
    if (m_hWnd == NULL)
    {
        return false;
    }

    SetWindowLongPtrW(m_hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    ShowWindow(m_hWnd, SW_SHOW);
    UpdateWindow(m_hWnd);
    return true;
}

// ----------------------------------------------------------------------------
// 初始化 OCCT 可视化对象链：驱动 -> 查看器 -> 视图(+窗口) -> 交互上下文
bool COCCTViewer::initOCCT()
{
    m_driver = new OpenGl_GraphicDriver(m_display);     // Windows 下显示连接为空

    m_viewer = new V3d_Viewer(m_driver);
    m_viewer->SetDefaultLights();                       // 默认灯光（头灯 + 环境光）
    m_viewer->SetLightOn();

    m_view = m_viewer->CreateView();

    m_ntWindow = new WNT_Window(reinterpret_cast<Aspect_Handle>(m_hWnd));
    m_view->SetWindow(m_ntWindow);
    if (!m_ntWindow->IsMapped())
    {
        m_ntWindow->Map();
    }

    // 深灰渐变背景
    m_view->SetBgGradientColors(Quantity_Color(0.28, 0.31, 0.38, Quantity_TOC_RGB),
                                Quantity_Color(0.10, 0.11, 0.14, Quantity_TOC_RGB),
                                Aspect_GradientFillMethod_Vertical,
                                Standard_False);

    m_context = new AIS_InteractiveContext(m_viewer);
    m_context->SetDisplayMode(AIS_Shaded, Standard_False);  // 默认着色模式
    return true;
}

// ----------------------------------------------------------------------------
// 释放视图与窗口资源（先于 HWND 销毁调用，断开 OCCT 与 HWND 的关联）
void COCCTViewer::destroy()
{
    if (!m_view.IsNull())
    {
        m_view->Remove();       // 视图与窗口解绑
    }
    m_displayed.clear();
    m_context.Nullify();
    m_view.Nullify();
    m_viewer.Nullify();
    m_ntWindow.Nullify();
    m_driver.Nullify();
    m_display.Nullify();

    if (m_hWnd != NULL)
    {
        HWND hWnd = m_hWnd;
        m_hWnd = NULL;          // 先清空，WM_DESTROY 中不再重复处理
        DestroyWindow(hWnd);
    }
}

// ----------------------------------------------------------------------------
void COCCTViewer::DisplayShape(const TopoDS_Shape& shape)
{
    DisplayShape(shape, Quantity_Color(Quantity_NOC_STEELBLUE), AIS_Shaded);
}

// ----------------------------------------------------------------------------
void COCCTViewer::DisplayShape(const TopoDS_Shape& shape, const Quantity_Color& color,
                               AIS_DisplayMode mode)
{
    if (shape.IsNull() || m_context.IsNull())
    {
        return;
    }

    Handle(AIS_Shape) aisShape = new AIS_Shape(shape);
    aisShape->SetColor(color);
    aisShape->SetDisplayMode(mode);
    aisShape->Attributes()->SetFaceBoundaryDraw(Standard_True); // 着色时描边，轮廓更清晰
    m_context->Display(aisShape, Standard_False);               // 延迟刷新，由 FitAll/Redraw 统一触发
    m_displayed.push_back(aisShape);
}

// ----------------------------------------------------------------------------
// 批量显示：自动从调色板取色区分不同模型
void COCCTViewer::DisplayShapes(const std::vector<TopoDS_Shape>& shapes)
{
    static const Quantity_NameOfColor palette[] =
    {
        Quantity_NOC_STEELBLUE,
        Quantity_NOC_INDIANRED3,
        Quantity_NOC_DARKOLIVEGREEN3,
        Quantity_NOC_GOLD,
        Quantity_NOC_MEDIUMORCHID3,
        Quantity_NOC_CYAN3,
    };
    for (size_t i = 0; i < shapes.size(); ++i)
    {
        DisplayShape(shapes[i], Quantity_Color(palette[i % 6]), AIS_Shaded);
    }
}

// ----------------------------------------------------------------------------
// 显示 3D 文字标注：AIS_TextLabel 以屏幕朝向渲染，锚点为模型坐标
void COCCTViewer::DisplayText(const gp_Pnt& position, const TCollection_AsciiString& text,
                              const Quantity_Color& color, double height)
{
    if (text.IsEmpty() || m_context.IsNull())
    {
        return;
    }

    Handle(AIS_TextLabel) label = new AIS_TextLabel();
    label->SetText(TCollection_ExtendedString(text));
    label->SetPosition(position);
    label->SetColor(color);
    label->SetHeight(height);
    label->SetZoomable(Standard_False);   // 固定像素高度，不随缩放变化
    // 放到 Topmost 图层（独立深度缓冲），文字始终绘制在模型之上，不会被 box 遮挡
    label->SetZLayer(Graphic3d_ZLayerId_Topmost);
    m_context->Display(label, Standard_False);
}

// ----------------------------------------------------------------------------
void COCCTViewer::RemoveAll()
{
    if (m_context.IsNull())
    {
        return;
    }
    m_context->EraseAll(Standard_False);
    m_displayed.clear();
}

// ----------------------------------------------------------------------------
void COCCTViewer::FitAll()
{
    if (m_view.IsNull())
    {
        return;
    }
    m_view->FitAll();
    m_view->Redraw();
}

// ----------------------------------------------------------------------------
void COCCTViewer::Redraw()
{
    if (!m_view.IsNull())
    {
        m_view->Redraw();
    }
}

// ----------------------------------------------------------------------------
// 切换全部已显示对象的显示模式（着色 / 线框）
void COCCTViewer::SetDisplayModeAll(AIS_DisplayMode mode)
{
    if (m_context.IsNull())
    {
        return;
    }
    for (std::vector<Handle(AIS_Shape)>::iterator it = m_displayed.begin(); it != m_displayed.end(); ++it)
    {
        m_context->SetDisplayMode(*it, mode, Standard_False);
    }
    m_context->UpdateCurrentViewer();
}

// ----------------------------------------------------------------------------
void COCCTViewer::SetOrientation(V3d_TypeOfOrientation orient)
{
    if (m_view.IsNull())
    {
        return;
    }
    m_view->SetProj(orient);
    FitAll();
}

// ----------------------------------------------------------------------------
// 模态消息循环：窗口关闭后返回
int COCCTViewer::Run()
{
    if (m_hWnd == NULL)
    {
        return -1;
    }

    MSG msg = {};
    while (m_hWnd != NULL)
    {
        const BOOL ret = GetMessageW(&msg, NULL, 0, 0);
        if (ret <= 0)
        {
            break;              // 0 = WM_QUIT，-1 = 错误
        }
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return static_cast<int>(msg.wParam);
}

// ----------------------------------------------------------------------------
// 静态窗口过程：把 HWND 映射回 COCCTViewer 实例后分发
LRESULT CALLBACK COCCTViewer::wndProcThunk(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    COCCTViewer* self = reinterpret_cast<COCCTViewer*>(GetWindowLongPtrW(hWnd, GWLP_USERDATA));
    if (self == NULL)
    {
        return DefWindowProcW(hWnd, msg, wParam, lParam);
    }
    return self->onMessage(hWnd, msg, wParam, lParam);
}

// ----------------------------------------------------------------------------
// 窗口消息处理
LRESULT COCCTViewer::onMessage(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        BeginPaint(hWnd, &ps);
        if (!m_view.IsNull())
        {
            m_view->Redraw();
        }
        EndPaint(hWnd, &ps);
        return 0;
    }
    case WM_ERASEBKGND:
        return 1;               // 禁止系统擦背景，OpenGL 自行清屏

    case WM_SIZE:
        onSize();
        return 0;

    case WM_LBUTTONDOWN:
        SetCapture(hWnd);
        onLButtonDown(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;
    case WM_LBUTTONUP:
        ReleaseCapture();
        onLButtonUp(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
        return 0;
    case WM_RBUTTONDOWN:
        SetCapture(hWnd);
        m_rDown = true;
        m_lastX = GET_X_LPARAM(lParam);
        m_lastY = GET_Y_LPARAM(lParam);
        return 0;
    case WM_RBUTTONUP:
        ReleaseCapture();
        m_rDown = false;
        return 0;
    case WM_MBUTTONDOWN:
        SetCapture(hWnd);
        m_mDown = true;
        m_lastX = GET_X_LPARAM(lParam);
        m_lastY = GET_Y_LPARAM(lParam);
        return 0;
    case WM_MBUTTONUP:
        ReleaseCapture();
        m_mDown = false;
        return 0;
    case WM_MOUSEMOVE:
        onMouseMove(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), wParam);
        return 0;
    case WM_MOUSEWHEEL:
        onMouseWheel(GET_WHEEL_DELTA_WPARAM(wParam));
        return 0;
    case WM_KEYDOWN:
        onKeyDown(wParam);
        return 0;

    case WM_CLOSE:
        destroy();              // 先释放 OCCT 资源，内部再 DestroyWindow
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

// ----------------------------------------------------------------------------
void COCCTViewer::onLButtonDown(int x, int y)
{
    m_lDown = true;
    m_lastX = x;
    m_lastY = y;
    if (!m_view.IsNull())
    {
        m_view->StartRotation(x, y);
    }
}

// ----------------------------------------------------------------------------
void COCCTViewer::onLButtonUp(int x, int y)
{
    m_lDown = false;
    m_lastX = x;
    m_lastY = y;
}

// ----------------------------------------------------------------------------
// 鼠标移动：左键旋转，右键 / 中键平移
void COCCTViewer::onMouseMove(int x, int y, WPARAM /*keys*/)
{
    if (m_view.IsNull())
    {
        return;
    }

    if (m_lDown)
    {
        m_view->Rotation(x, y);
    }
    else if (m_rDown || m_mDown)
    {
        m_view->Pan(x - m_lastX, m_lastY - y);
    }

    m_lastX = x;
    m_lastY = y;
}

// ----------------------------------------------------------------------------
// 滚轮缩放：向上放大，向下缩小
void COCCTViewer::onMouseWheel(short delta)
{
    if (m_view.IsNull())
    {
        return;
    }
    const double factor = delta > 0 ? 1.15 : 1.0 / 1.15;
    m_view->SetZoom(factor, Standard_True);
}

// ----------------------------------------------------------------------------
// 快捷键：F 全图 / R 重置 / S 着色 / W 线框 / 1-4 标准视图 / Esc 退出
void COCCTViewer::onKeyDown(WPARAM key)
{
    switch (key)
    {
    case VK_ESCAPE:
        SendMessageW(m_hWnd, WM_CLOSE, 0, 0);
        break;
    case 'F':
        FitAll();
        break;
    case 'R':
        if (!m_view.IsNull())
        {
            m_view->Reset();
            FitAll();
        }
        break;
    case 'S':
        SetDisplayModeAll(AIS_Shaded);
        break;
    case 'W':
        SetDisplayModeAll(AIS_WireFrame);
        break;
    case '1':
        SetOrientation(V3d_Xpos);
        break;
    case '2':
        SetOrientation(V3d_Ypos);
        break;
    case '3':
        SetOrientation(V3d_Zpos);
        break;
    case '4':
        SetOrientation(V3d_XposYposZpos);
        break;
    default:
        break;
    }
}

// ----------------------------------------------------------------------------
void COCCTViewer::onSize()
{
    if (!m_view.IsNull())
    {
        m_view->MustBeResized();
        m_view->Redraw();
    }
}
