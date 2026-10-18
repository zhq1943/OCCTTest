# ProjectOCCT

基于 **Open CASCADE Technology (OCCT) 7.8.0** 的几何建模与轴测投影测试项目，演示任意方向包围盒、棱柱、扫掠体的生成、HLR 消隐投影以及 DXF 输出。

## 技术栈

- **语言**：C++14
- **几何内核**：OCCT 7.8.0（头文件位于 `include/opencascade/`，导入库与运行时 DLL 位于 `lib/`）
- **DXF 输出**：dxflib（头文件 `include/dxflib/`，库 `lib/dxflib.lib` / `lib/dxflib.dll`）
- **JSON 解析**：nlohmann/json（单头文件 `include/nlohmann/json.hpp`）
- **编译环境**：MSVC v142（Visual Studio 2019/2022），Debug|x64

## 目录结构

```
ProjectOCCT/
├── main.cpp              // 程序入口，仅调用各测试函数
├── Tests.cpp / Tests.h   // 全部测试函数（建模 + 投影 + DXF）
├── Shape.cpp / Shape.h   // 几何定义类（CShape_Box/Prism/Sweep，OCCT 移植版，头文件内联实现）
├── ProjectHelper.cpp/.h  // 投影与建模辅助（BuildViewTransform、HLR 投影、BuildFaceWithHoles 等）
├── ProjectOCCT.sln       // Visual Studio 解决方案
├── ProjectOCCT.vcxproj   // 项目文件（含 PostBuildEvent：复制 lib/*.dll 到输出目录）
├── include/              // 第三方头文件（opencascade / dxflib / nlohmann）
├── lib/                  // OCCT 7.8.0 与 dxflib 的 .lib 导入库及运行时 .dll
├── test*.txt             // 测试输入数据（截面轮廓 / 扫掠路径 / JSON 线框定义）
├── .gitignore
└── README.md
```

## 构建与运行

1. 用 Visual Studio 打开 `ProjectOCCT.sln`
2. 选择 **Debug | x64** 配置
3. 生成解决方案（`Build > Build Solution`）
4. 运行 `x64/Debug/ProjectOCCT.exe`

构建后 `lib/` 中的全部 DLL 会通过 PostBuildEvent 自动复制到输出目录，保证运行时依赖完整。

## 测试内容

程序按顺序执行以下测试，结果输出到控制台，并在 `x64/Debug/` 生成对应 DXF：

| 测试函数 | 说明 | 输出 DXF |
|---|---|---|
| `OCCT_Test` | 基础 OCCT 几何原语测试 | — |
| `BuildHollowPrismTest` | 带孔棱柱（`BuildFaceWithHoles` + `MakePrism`） | — |
| `BuildFaceOnInclinedPlaneTest` | 倾斜平面真实数据建面 | — |
| `LineSweepDXFTest` | L 型脊线扫掠 + 轴测 DXF | — |
| `UnorderedPathSweepDXFTest` | 乱序轨迹段重排后扫掠 | — |
| `JsonWireSweepTest` | 解析 `test3.txt` 的 JSON 扫掠条目，逐条建模汇总 | `sweep_json_wire.dxf` |
| `JsonWireTest2` | 解析 `test4.txt` 的 Prism/Sweep/Box 混合条目 | `wire_test_Box_Dxf.dxf` |
| `ShapeClassTest` | 通过 `CShape_*` 接口取数生成任意方向 Box/Prism/Sweep | `shape_class_test.dxf` |
| `CombineBoxTest` | `CShape_Box::CombineBox` 合并（远离/相交/包含/任意方向） | `combine_box_test.dxf` |

## Shape 类定义

`Shape.h` 将遗留 COM/MFC 风格的几何定义类移植为 OCCT 值类型版本：

- `IXYZ` → `gp_Pnt`，`IVector` → `gp_Vec`，`IPlane` → `gp_Pln`，`CString` → `std::string`
- `CComPtr` / `Clone` / `GetCopy` → 值拷贝（OCCT 几何类型为值语义）
- `CShape_Box`：中心点 + 三个带长度（半尺寸）的方向向量；支持 `GetMinPt`、`GetBoxVertices`、`CombineBox`（轴对齐包围盒合并）等
- `CShape_Prism`：轮廓折线 + 面内/法向三方向 + 厚度 + 孔洞
- `CShape_Sweep`：轨迹 + 截面轮廓 + 截面偏移 + 起终斜切角

## 编码约定

- 源文件统一使用 **UTF-8 with BOM** 编码（MSVC 无 BOM 时会按系统代码页读取含中文的源文件，导致编译错误）
- 2D 曲线投影区分解析形式（线段/圆弧/圆）与离散补充折线
- `BuildViewTransform` 是世界→视图变换的唯一权威来源，HLR 投影与脊线投影共用
