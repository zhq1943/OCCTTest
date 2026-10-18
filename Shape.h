#pragma once

// ============================================================================
// Shape.h —— 遗留 COM/MFC 版 Shape 定义的 OCCT 移植版
//
// 未定义数据类型 -> OCCT 类型 映射关系：
//   IXYZ    (三维点)   -> gp_Pnt
//   IVector (三维向量) -> gp_Vec
//   IPlane  (平面)     -> gp_Pln
//   CString (字符串)   -> std::string
//   DOUBLE             -> double
//   BOOL               -> bool
//   IMatrix (变换矩阵) -> gp_Trsf
//   Json::Value        -> nlohmann::json
//   ICxGeCurveColl(曲线集合) -> CGeCurveColl（折线点集容器，值语义）
//   CComPtr<T> COM 智能指针 -> 值成员（OCCT 几何类型均为值类型，拷贝即 Clone）
// ============================================================================

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <cstdio>

// ----------------------------------------------------------------------------
// CGeCurveColl：曲线集合（ICxGeCurveColl 的替代）。
// 每条曲线表示为一串折线点；扫掠轨迹/截面轮廓等单折线场景使用第 0 条曲线。
// 值语义：拷贝构造/赋值即完成原 COM 接口 GetCopy 的深拷贝语义。
// ----------------------------------------------------------------------------
class CGeCurveColl
{
public:
    std::vector<std::vector<gp_Pnt>> curves; // 每条曲线 = 一串折线点

    CGeCurveColl() = default;
    explicit CGeCurveColl(const std::vector<gp_Pnt>& polyline) { curves.push_back(polyline); }

    void   AddCurve(const std::vector<gp_Pnt>& polyline) { curves.push_back(polyline); }
    size_t CurveCount() const { return curves.size(); }
    bool   IsEmpty() const { return curves.empty(); }

    const std::vector<gp_Pnt>& Curve(size_t i) const { return curves[i]; }

    // 便捷访问：单折线场景（扫掠轨迹/截面轮廓）的点集
    const std::vector<gp_Pnt>& Points() const { return curves.front(); }
    void SetPoints(const std::vector<gp_Pnt>& pts)
    {
        curves.clear();
        curves.push_back(pts);
    }
};

// ----------------------------------------------------------------------------
// CShape：基类
// ----------------------------------------------------------------------------
class CShape
{
public:
    CShape() = default;
    virtual ~CShape() = default;

    virtual int GetType() = 0;

    void SetName(const std::string& tmpName) { msName = tmpName; }
    const std::string& GetName() const { return msName; }

private:
    std::string msName;
};

// ----------------------------------------------------------------------------
// CShape_Sweep：扫掠定义（轨迹 + 截面轮廓 + 起终切角 + 截面偏移）
// ----------------------------------------------------------------------------
class CShape_Sweep : public CShape
{
public:
    CShape_Sweep()
        : mdOffset(0.0), mbResetPos(false), mdStaAng(0.0), mdEndAng(0.0), mbArcToLn(false)
    {
    }

    CShape_Sweep(const CGeCurveColl& geCollSweep,
                 const CGeCurveColl& geCollLoop,
                 double dOffset)
        : mGeCollSweep(geCollSweep), mGeCollLoop(geCollLoop), mdOffset(dOffset),
          mbResetPos(false), mdStaAng(0.0), mdEndAng(0.0), mbArcToLn(false)
    {
    }

    int GetType() override { return 3; }

private:
    CGeCurveColl mGeCollSweep;  // 轨迹
    CGeCurveColl mGeCollLoop;   // 截面轮廓
    double       mdOffset;      // 截面沿路径偏移
    gp_Vec       mSweepDir;     // 轨迹所在平面法向（零向量=未设置）
    gp_Vec       mSectionDir;   // 截面所在曲线法向（零向量=未设置）
    bool         mbResetPos;
    double       mdStaAng;      // 起始斜切角
    double       mdEndAng;      // 终点斜切角
    bool         mbArcToLn;     // 线段化圆弧

public:
    void SetSEAng(double dStaAng, double dEndAng) { mdStaAng = dStaAng; mdEndAng = dEndAng; }
    void GetSEAng(double& dStaAng, double& dEndAng) const { dStaAng = mdStaAng; dEndAng = mdEndAng; }

    void SetResetPos(bool bReset) { mbResetPos = bReset; }
    void GetResetPos(bool& bReset) const { bReset = mbResetPos; }

    void SetSweepDir(const gp_Vec& dir) { mSweepDir = dir; }
    void SetSeciontDir(const gp_Vec& dir) { mSectionDir = dir; }

    // 未设置时返回零向量（原 COM 版本返回空指针）
    gp_Vec GetSweepDir() const { return mSweepDir; }
    gp_Vec GetSectionDir() const { return mSectionDir; }

    bool HasSweepDir() const { return mSweepDir.Magnitude() > 1.0e-12; }
    bool HasSectionDir() const { return mSectionDir.Magnitude() > 1.0e-12; }

    // 取出轨迹与截面轮廓（值拷贝，等同原 GetCopy 语义）
    void GetInfo(CGeCurveColl& outGeCollSweep, CGeCurveColl& outGeCollLoop) const
    {
        outGeCollSweep = mGeCollSweep;
        outGeCollLoop  = mGeCollLoop;
    }

    double GetOffset() const { return mdOffset; }
};

// ----------------------------------------------------------------------------
// CShape_Prism：棱柱定义（轮廓 + 面内/法向三方向 + 厚度 + 孔洞）
// ----------------------------------------------------------------------------
class CShape_Prism : public CShape
{
public:
    CShape_Prism() : mdThick(0.0) {}

    CShape_Prism(const CGeCurveColl& geColl,
                 const gp_Vec& dirX,
                 const gp_Vec& dirY,
                 const gp_Vec& dirZ,
                 double dThick)
        : mGeCollLoop(geColl), mDirX(dirX), mDirY(dirY), mDirZ(dirZ), mdThick(dThick)
    {
    }

    int GetType() override { return 2; }

private:
    std::vector<CGeCurveColl> mvecGeCollHoles; // 孔洞
    CGeCurveColl mGeCollLoop;                  // 轮廓线

    gp_Vec mDirX; // 面内 x 方向向量
    gp_Vec mDirY; // 面内 y 方向向量
    gp_Vec mDirZ; // 面法向（拉伸方向）
    double mdThick; // 厚度

public:
    void GetGeColl(CGeCurveColl& outGeColl) const { outGeColl = mGeCollLoop; }

    // 输出归一化后的三方向（原接口 SetLength(1) 语义）
    void GetDir(gp_Vec& outDirX, gp_Vec& outDirY, gp_Vec& outDirZ) const
    {
        outDirX = mDirX.Normalized();
        outDirY = mDirY.Normalized();
        outDirZ = mDirZ.Normalized();
    }

    double GetThick() const { return mdThick; }

    void SetHole(const std::vector<CGeCurveColl>& vecHole) { mvecGeCollHoles = vecHole; }
    void GetHole(std::vector<CGeCurveColl>& outVecHole) const { outVecHole = mvecGeCollHoles; }
};

// ----------------------------------------------------------------------------
// CShape_Box：任意方向包围盒（中心点 + 三个带长度的方向向量）
// 约定：mDirX/Y/Z 的【向量长度 = 该方向半尺寸】，方向为盒轴正方向；
//       盒 8 顶点 = center ± dirX ± dirY ± dirZ。
// ----------------------------------------------------------------------------
class CShape_Box : public CShape
{
public:
    CShape_Box() : mHasPlane(false) {}

    CShape_Box(const gp_Pnt& cent,
               const gp_Vec& dirX,
               const gp_Vec& dirY,
               const gp_Vec& dirZ)
        : mCenter(cent), mDirX(dirX), mDirY(dirY), mDirZ(dirZ), mHasPlane(false)
    {
    }

    int GetType() override { return 1; }

private:
    gp_Pnt mCenter;  // 中心点
    gp_Vec mDirX;    // x 方向向量（长度=半尺寸）
    gp_Vec mDirY;    // y 方向向量（长度=半尺寸）
    gp_Vec mDirZ;    // z 方向向量（长度=半尺寸）

    gp_Pln mPlane;   // 平面（可选）
    bool   mHasPlane;

public:
    gp_Pnt GetCenter() const { return mCenter; }

    bool isValid() const
    {
        return mDirX.Magnitude() > 1.0e-12
            && mDirY.Magnitude() > 1.0e-12
            && mDirZ.Magnitude() > 1.0e-12;
    }

    // 最小角点 = 中心 - 三个半长向量
    gp_Pnt GetMinPt() const
    {
        return mCenter.Translated(-mDirX - mDirY - mDirZ);
    }

    void SetPlane(const gp_Pln& pln) { mPlane = pln; mHasPlane = true; }
    bool HasPlane() const { return mHasPlane; }
    const gp_Pln& GetPlane() const { return mPlane; }

    // 输出归一化后的三轴方向
    void GetDir(gp_Vec& outDirX, gp_Vec& outDirY, gp_Vec& outDirZ) const
    {
        outDirX = mDirX.Normalized();
        outDirY = mDirY.Normalized();
        outDirZ = mDirZ.Normalized();
    }

    // 三向全尺寸 = 2 × 半长
    void GetSize(double& dX, double& dY, double& dZ) const
    {
        dX = mDirX.Magnitude() * 2.0;
        dY = mDirY.Magnitude() * 2.0;
        dZ = mDirZ.Magnitude() * 2.0;
    }

    // 同向同尺寸判定（容差 1e-6）
    bool isSame(const CShape_Box& tmp) const
    {
        const double tol = 1.0e-6;
        if (mCenter.Distance(tmp.mCenter) > tol) return false;
        if ((mDirX - tmp.mDirX).Magnitude() > tol) return false;
        if ((mDirY - tmp.mDirY).Magnitude() > tol) return false;
        if ((mDirZ - tmp.mDirZ).Magnitude() > tol) return false;
        return true;
    }

    // 值拷贝（等同原 GetCopy/Clone 语义）
    void GetCopy(CShape_Box& outCopy) const { outCopy = *this; }

    // 施加刚体/仿射变换：中心按点变换，方向按向量变换（不含平移）
    void Trans(const gp_Trsf& mat)
    {
        mCenter.Transform(mat);
        mDirX.Transform(mat);
        mDirY.Transform(mat);
        mDirZ.Transform(mat);
        if (mHasPlane)
            mPlane.Transform(mat);
    }

    // 导出 JSON 描述
    void GetBoxInfo_Json(nlohmann::json& boxInfo) const
    {
        double sx, sy, sz;
        GetSize(sx, sy, sz);
        boxInfo["Center"] = { {"x", mCenter.X()}, {"y", mCenter.Y()}, {"z", mCenter.Z()} };
        boxInfo["DirX"]   = { {"x", mDirX.X()}, {"y", mDirX.Y()}, {"z", mDirX.Z()} };
        boxInfo["DirY"]   = { {"x", mDirY.X()}, {"y", mDirY.Y()}, {"z", mDirY.Z()} };
        boxInfo["DirZ"]   = { {"x", mDirZ.X()}, {"y", mDirZ.Y()}, {"z", mDirZ.Z()} };
        boxInfo["Size"]   = { {"x", sx}, {"y", sy}, {"z", sz} };
    }

    // 导出调试文本
    void GetBoxInfo_Debug(std::string& out) const
    {
        char buf[256];
        out = "\nBox\n";
        std::snprintf(buf, sizeof(buf), "Center[%.2f, %.2f, %.2f]\n", mCenter.X(), mCenter.Y(), mCenter.Z());
        out += buf;
        std::snprintf(buf, sizeof(buf), "DirX[%.2f, %.2f, %.2f]\n", mDirX.X(), mDirX.Y(), mDirX.Z());
        out += buf;
        std::snprintf(buf, sizeof(buf), "DirY[%.2f, %.2f, %.2f]\n", mDirY.X(), mDirY.Y(), mDirY.Z());
        out += buf;
        std::snprintf(buf, sizeof(buf), "DirZ[%.2f, %.2f, %.2f]\n", mDirZ.X(), mDirZ.Y(), mDirZ.Z());
        out += buf;
    }

    // 取与输入方向最接近的半长向量（带符号，沿该轴正/负向）
    gp_Vec GetHalfDirByDir(const gp_Vec& dir) const
    {
        const gp_Vec axes[3] = { mDirX, mDirY, mDirZ };
        double bestDot = -1.0;
        gp_Vec best = mDirX;
        for (const gp_Vec& ax : axes)
        {
            double m = ax.Magnitude();
            if (m < 1.0e-12) continue;
            double d = dir.Dot(ax) / m; // 与单位轴的点积（保留符号信息）
            if (std::fabs(d) > bestDot)
            {
                bestDot = std::fabs(d);
                best = (d >= 0.0) ? ax : -ax;
            }
        }
        return best;
    }

    // 求 tmp 三轴到本盒三轴的置换映射：mapAxis[i] = 与本盒第 i 轴平行的 tmp 轴号(0/1/2)。
    // 任一轴找不到近似平行（|cos| > 1e-3）的对应轴时返回 false。
    bool FindAxisPermutation(const CShape_Box& tmp, int mapAxis[3]) const
    {
        const gp_Vec selfAx[3] = { mDirX.Normalized(), mDirY.Normalized(), mDirZ.Normalized() };
        const gp_Vec tmpAx[3]  = { tmp.mDirX.Normalized(), tmp.mDirY.Normalized(), tmp.mDirZ.Normalized() };
        bool used[3] = { false, false, false };
        for (int i = 0; i < 3; ++i)
        {
            double best = -1.0;
            int bestJ = -1;
            for (int j = 0; j < 3; ++j)
            {
                if (used[j]) continue;
                double d = std::fabs(selfAx[i].Dot(tmpAx[j]));
                if (d > best) { best = d; bestJ = j; }
            }
            if (bestJ < 0 || best < 1.0e-3) return false;
            mapAxis[i] = bestJ;
            used[bestJ] = true;
        }
        return true;
    }

    // 按置换映射重排 tmp 半长向量，使方向与本盒轴正向一致（点积为负则取反）
    bool RemapTmpAxes(const CShape_Box& tmp, const int mapAxis[3],
                      gp_Vec& outRx, gp_Vec& outRy, gp_Vec& outRz) const
    {
        const gp_Vec tmpAx[3]  = { tmp.mDirX, tmp.mDirY, tmp.mDirZ };
        const gp_Vec selfAx[3] = { mDirX.Normalized(), mDirY.Normalized(), mDirZ.Normalized() };
        gp_Vec* out[3] = { &outRx, &outRy, &outRz };
        for (int i = 0; i < 3; ++i)
        {
            int j = mapAxis[i];
            if (j < 0 || j > 2) return false;
            gp_Vec v = tmpAx[j];
            if (v.Dot(selfAx[i]) < 0.0) v.Reverse();
            *out[i] = v;
        }
        return true;
    }

    // 合并一个与本盒同向的盒：在本盒轴系下取两盒 8 顶点的包围范围，改写本盒
    bool MergeOrientedBox(const CShape_Box& alignedTmpBox)
    {
        std::vector<gp_Pnt> vertsA, vertsB;
        GetBoxVertices(vertsA);
        alignedTmpBox.GetBoxVertices(vertsB);

        const gp_Vec ax = mDirX.Normalized();
        const gp_Vec ay = mDirY.Normalized();
        const gp_Vec az = mDirZ.Normalized();

        double mn[3] = { 1.0e30, 1.0e30, 1.0e30 };
        double mx[3] = { -1.0e30, -1.0e30, -1.0e30 };
        for (int pass = 0; pass < 2; ++pass)
        {
            const std::vector<gp_Pnt>& vs = pass == 0 ? vertsA : vertsB;
            for (const gp_Pnt& p : vs)
            {
                gp_Vec r(mCenter, p);
                double c[3] = { r.Dot(ax), r.Dot(ay), r.Dot(az) };
                for (int k = 0; k < 3; ++k)
                {
                    if (c[k] < mn[k]) mn[k] = c[k];
                    if (c[k] > mx[k]) mx[k] = c[k];
                }
            }
        }
        gp_Vec offset = ax * ((mn[0] + mx[0]) * 0.5)
                      + ay * ((mn[1] + mx[1]) * 0.5)
                      + az * ((mn[2] + mx[2]) * 0.5);
        mCenter = mCenter.Translated(offset);
        mDirX = ax * ((mx[0] - mn[0]) * 0.5);
        mDirY = ay * ((mx[1] - mn[1]) * 0.5);
        mDirZ = az * ((mx[2] - mn[2]) * 0.5);
        return true;
    }

    // 合并任意盒：先求轴置换并重排，再按同向合并
    bool CombineBox(const CShape_Box& tmp)
    {
        int mapAxis[3];
        if (!FindAxisPermutation(tmp, mapAxis)) return false;
        gp_Vec rx, ry, rz;
        if (!RemapTmpAxes(tmp, mapAxis, rx, ry, rz)) return false;
        CShape_Box aligned(tmp.mCenter, rx, ry, rz);
        return MergeOrientedBox(aligned);
    }

    // 8 顶点 = center ± dirX ± dirY ± dirZ
    bool GetBoxVertices(std::vector<gp_Pnt>& outVerts) const
    {
        if (!isValid()) return false;
        outVerts.clear();
        outVerts.reserve(8);
        for (int i = -1; i <= 1; i += 2)
            for (int j = -1; j <= 1; j += 2)
                for (int k = -1; k <= 1; k += 2)
                    outVerts.push_back(mCenter.Translated(mDirX * i + mDirY * j + mDirZ * k));
        return true;
    }

    // 12 条棱边（每边 2 个端点，存入曲线集合）
    bool GetBoxEdges(CGeCurveColl& outGeColl) const
    {
        std::vector<gp_Pnt> v;
        if (!GetBoxVertices(v)) return false;
        // 顶点序：idx = (i+1)/2*4 + (j+1)/2*2 + (k+1)/2（i:X, j:Y, k:Z）
        static const int edges[12][2] = {
            {0,1},{2,3},{4,5},{6,7},   // 沿 Z
            {0,2},{1,3},{4,6},{5,7},   // 沿 Y
            {0,4},{1,5},{2,6},{3,7}    // 沿 X
        };
        outGeColl.curves.clear();
        for (const auto& e : edges)
            outGeColl.AddCurve({ v[e[0]], v[e[1]] });
        return true;
    }
};
