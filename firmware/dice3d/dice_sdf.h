// ============================================================================
//  dice_sdf.h —— 骰子的 SDF 光线步进渲染核心（纯 C++，不依赖 Arduino）
//
//  和 preview/index.html 里那套是同一套数学，但做了三处关键优化：
//
//  ① 射线只变换一次到「物体空间」
//     JS 版每个步进都做一次四元数旋转（56 步 = 56 次旋转/像素）。
//     这里把射线起点和方向各旋转一次，之后直接在物体空间步进 —— 省掉 55 次。
//
//  ② 包围球在物体空间里是固定半径 H*sqrt(3)，不用每帧算
//
//  ③ 输出 RGB565（设备上直接用），不是 RGBA32
//
//  能在 Mac 上单独编译（见 host_test.cpp），这样每次改渲染都能先离线验证
//  正确性和耗时，再烧板子。
// ============================================================================
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

namespace dice {

// ---------------------------------------------------------------- 参数
struct Params {
  // 几何（单位：骰子边长 = 1.0）
  float H         = 0.5f;     // 半边长
  float R         = 0.088f;   // 圆角半径（16mm 骰子约 1.4mm ≈ 0.088）
  float pipOff    = 0.29f;    // 点数在面内的偏移
  float pipRad    = 0.098f;   // 点数半径
  float pipDepth  = 0.34f;    // 坑深 / 点数半径

  // 相机
  float focal     = 1100.0f;  // 焦距（按 480 画布为基准）
  float camZ      = 4.2f;     // 相机距离

  // 步进
  int   maxSteps  = 56;
  float hitEps    = 0.0008f;
  float stepScale = 0.92f;    // <1 更稳但更慢

  // 颜色（0-255）
  uint8_t bodyR = 214, bodyG = 38, bodyB = 38;      // 红底
  uint8_t pipR  = 252, pipG  = 250, pipB  = 246;    // 白点

  // 光照方向（由表面指向光源，已归一化）
  float lx = -0.38f, ly = 0.70f, lz = -0.60f;
};

// ---------------------------------------------------------------- 基础数学
struct V3 { float x, y, z; };
struct Quat { float w, x, y, z; };

static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3 vmul(V3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static inline float vdot(V3 a, V3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3 vcross(V3 a, V3 b) {
  return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline float vlen(V3 a) { return sqrtf(vdot(a, a)); }
static inline V3 vnorm(V3 a) {
  float l = vlen(a);
  return (l > 1e-9f) ? vmul(a, 1.0f/l) : v3(0,0,0);
}

static inline Quat qmul(Quat a, Quat b) {
  Quat r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}
static inline Quat qnorm(Quat q) {
  float l = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (l < 1e-9f) { Quat i = {1,0,0,0}; return i; }
  float s = 1.0f/l;
  Quat r = {q.w*s, q.x*s, q.y*s, q.z*s};
  return r;
}
static inline Quat qaxis(V3 ax, float ang) {
  V3 a = vnorm(ax);
  float h = ang*0.5f, s = sinf(h);
  Quat r = {cosf(h), a.x*s, a.y*s, a.z*s};
  return r;
}
static inline Quat qconj(Quat q) { Quat r = {q.w, -q.x, -q.y, -q.z}; return r; }
/** 把单位向量 a 转到 b 的最短弧 */
static inline Quat qfromto(V3 a, V3 b) {
  a = vnorm(a); b = vnorm(b);
  float d = vdot(a, b);
  if (d > 0.99999f) { Quat i = {1,0,0,0}; return i; }
  if (d < -0.99999f) {
    V3 ax = vcross(a, v3(1,0,0));
    if (vlen(ax) < 1e-4f) ax = vcross(a, v3(0,1,0));
    ax = vnorm(ax);
    Quat r = {0, ax.x, ax.y, ax.z};
    return r;
  }
  V3 ax = vcross(a, b);
  Quat r = {1.0f + d, ax.x, ax.y, ax.z};
  return qnorm(r);
}
static inline V3 qrot(V3 v, Quat q) {
  V3 u = v3(q.x, q.y, q.z);
  V3 t = vmul(vcross(u, v), 2.0f);
  return vadd(vadd(v, vmul(t, q.w)), vcross(u, t));
}

// ---------------------------------------------------------------- 六个面
// 对面和 = 7；西方标准手性：1 朝前(+Z)、2 朝上(+Y) → 4 在右(+X)、3 在左(-X)
struct FaceAxis { V3 n, r, u; uint8_t val; };
static const FaceAxis FACES[6] = {
  { { 0, 0, 1}, { 1, 0, 0}, {0, 1, 0}, 1 },  // +Z
  { { 0, 0,-1}, {-1, 0, 0}, {0, 1, 0}, 6 },  // -Z
  { { 1, 0, 0}, { 0, 0,-1}, {0, 1, 0}, 4 },  // +X
  { {-1, 0, 0}, { 0, 0, 1}, {0, 1, 0}, 3 },  // -X
  { { 0, 1, 0}, { 1, 0, 0}, {0, 0,-1}, 2 },  // +Y
  { { 0,-1, 0}, { 1, 0, 0}, {0, 0, 1}, 5 },  // -Y
};

// 每个点数的面内 2D 坐标（最多 6 个）
static const float PIP_XY[7][6][2] = {
  {},
  {{0,0}},
  {{-0.29f,-0.29f},{0.29f,0.29f}},
  {{-0.29f,-0.29f},{0,0},{0.29f,0.29f}},
  {{-0.29f,-0.29f},{0.29f,-0.29f},{-0.29f,0.29f},{0.29f,0.29f}},
  {{-0.29f,-0.29f},{0.29f,-0.29f},{0,0},{-0.29f,0.29f},{0.29f,0.29f}},
  {{-0.29f,-0.29f},{0.29f,-0.29f},{-0.29f,0},{0.29f,0},{-0.29f,0.29f},{0.29f,0.29f}},
};

// ---------------------------------------------------------------- RGB565
static inline uint16_t rgb565(int r, int g, int b) {
  if (r < 0) r = 0; if (r > 255) r = 255;
  if (g < 0) g = 0; if (g > 255) g = 255;
  if (b < 0) b = 0; if (b > 255) b = 255;
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// ---------------------------------------------------------------- 渲染器
class Renderer {
public:
  Params p;
  Quat   qInv;              // 世界 -> 物体
  float  centerY = 0;
  int    W = 0, H = 0;
  uint16_t *buf = nullptr;

  // 本帧骰子的像素包围盒（渲染分辨率坐标，已裁到画布内）
  int bboxX0 = 0, bboxY0 = 0, bboxX1 = 0, bboxY1 = 0;

  // 统计（给性能测试用）
  uint32_t statPixels = 0;   // 实际步进的像素数
  uint32_t statHits   = 0;   // 命中数
  uint32_t statSteps  = 0;   // 总步进次数
  uint32_t statSdf    = 0;   // SDF 求值总次数

  inline float sdfBox(V3 q) const {
    const float BOX = p.H - p.R;
    float qx = fabsf(q.x) - BOX;
    float qy = fabsf(q.y) - BOX;
    float qz = fabsf(q.z) - BOX;
    float mx = qx > 0 ? qx : 0, my = qy > 0 ? qy : 0, mz = qz > 0 ? qz : 0;
    float outside = sqrtf(mx*mx + my*my + mz*mz);
    float imax = qx > qy ? qx : qy;
    if (qz > imax) imax = qz;
    float inside = imax < 0 ? imax : 0;
    return outside + inside - p.R;
  }

  // 四面体梯度法线
  V3 sdfNormal(V3 q) {
    static const float TET[4][3] = {
      { 1,-1,-1}, {-1,-1, 1}, {-1, 1,-1}, { 1, 1, 1}
    };
    const float e = 0.0006f;
    float nx = 0, ny = 0, nz = 0;
    for (int i = 0; i < 4; i++) {
      V3 t = v3(q.x + TET[i][0]*e, q.y + TET[i][1]*e, q.z + TET[i][2]*e);
      float d = sdfBox(t);
      nx += TET[i][0]*d; ny += TET[i][1]*d; nz += TET[i][2]*d;
    }
    statSdf += 4;
    return vnorm(v3(nx, ny, nz));
  }

  // 点数坑判定：命中返回 true 并给出坑法线（物体空间）与坑沿系数
  bool pipHit(const FaceAxis &ax, float sign, float pu, float pv, int val,
              V3 &nOut, float &openness, float &tEdge) const {
    const float r2 = p.pipRad * p.pipRad;
    const float (*pts)[2] = PIP_XY[val];
    for (int i = 0; i < val; i++) {
      float da = pu - pts[i][0];
      float db = pv - pts[i][1];
      float d2 = da*da + db*db;
      if (d2 >= r2) continue;
      float d = sqrtf(d2);
      float t = d / p.pipRad;
      float s = sqrtf(fmaxf(1e-6f, 1.0f - t*t));
      float slope = p.pipDepth * t / fmaxf(1e-3f, s);
      float ux = d > 1e-9f ? da/d : 0;
      float uy = d > 1e-9f ? db/d : 0;
      float nl = sqrtf(slope*slope + 1.0f);
      float rx = slope*ux/nl, ry = slope*uy/nl, rz = 1.0f/nl;
      V3 nAxis = vmul(ax.n, sign);
      V3 radial = vadd(vmul(ax.r, rx), vmul(ax.u, ry));
      nOut = vnorm(vsub(vmul(nAxis, rz), radial));
      openness = rz;
      tEdge = t;
      return true;
    }
    return false;
  }

  /** 渲染一帧到 buf（W×H，RGB565）。q = 骰子姿态，yOff = 抛起高度。 */
  void render(Quat q, float yOff) {
    statPixels = statHits = statSteps = statSdf = 0;

    qInv = qconj(q);
    centerY = yOff;

    // 骰子世界中心 (0, yOff, camZ)
    const float cx = 0, cy = yOff, cz = p.camZ;
    // 物体空间里骰子中心在原点，包围球半径 = H*sqrt(3)
    const float bound = p.H * 1.7320508f;

    const float halfW = W * 0.5f, halfH = H * 0.5f;
    // 焦距按画布缩放：p.focal 是按 480 画布给的
    const float focal = p.focal * (W / 480.0f);

    // ---- 屏幕包围盒：投影 8 个角（只用于减少无效像素）
    float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
    for (int i = 0; i < 8; i++) {
      float sx = (i & 1) ? p.H : -p.H;
      float sy = (i & 2) ? p.H : -p.H;
      float sz = (i & 4) ? p.H : -p.H;
      V3 v = qrot(v3(sx, sy, sz), q);
      float wz = v.z + cz;
      if (wz <= 0.3f) continue;
      float px = halfW + focal * v.x / wz;
      float py = halfH - focal * (v.y + yOff) / wz;
      if (px < minX) minX = px; if (px > maxX) maxX = px;
      if (py < minY) minY = py; if (py > maxY) maxY = py;
    }
    int bx0 = (int)floorf(minX) - 2, bx1 = (int)ceilf(maxX) + 2;
    int by0 = (int)floorf(minY) - 2, by1 = (int)ceilf(maxY) + 2;
    if (bx0 < 0) bx0 = 0;
    if (by0 < 0) by0 = 0;
    if (bx1 > W - 1) bx1 = W - 1;
    if (by1 > H - 1) by1 = H - 1;

    // 只清包围盒（省掉全屏 memset）
    bboxX0 = bx0; bboxY0 = by0; bboxX1 = bx1; bboxY1 = by1;
    if (bx0 <= bx1 && by0 <= by1) {
      for (int yy = by0; yy <= by1; yy++)
        memset(buf + (size_t)yy * W + bx0, 0, (size_t)(bx1 - bx0 + 1) * 2);
    }

    // ★ 每帧只算一次：相机原点在物体空间的位置（原来每像素都重复算）
    const V3 orgObj = qrot(v3(-cx, -cy, -cz), qInv);

    // 阴影先画（可能超出骰子包围盒），画完把它的范围并进 bbox
    drawGroundShadow(q, yOff, focal);
    if (shX0 <= shX1 && shY0 <= shY1) {
      if (shX0 < bboxX0) bboxX0 = shX0;
      if (shY0 < bboxY0) bboxY0 = shY0;
      if (shX1 > bboxX1) bboxX1 = shX1;
      if (shY1 > bboxY1) bboxY1 = shY1;
    }

    // ---- 逐像素
    for (int y = by0; y <= by1; y++) {
      float syv = halfH - (y + 0.5f);
      for (int x = bx0; x <= bx1; x++) {
        float sxv = (x + 0.5f) - halfW;

        // 世界空间射线方向（相机在原点，朝 +Z）
        float dx = sxv / focal, dy = syv / focal, dz = 1.0f;
        float dl = sqrtf(dx*dx + dy*dy + dz*dz);
        dx /= dl; dy /= dl; dz /= dl;

        // 射线方向变换到物体空间（方向每像素不同，必须算）
        V3 dirObj = qrot(v3(dx, dy, dz), qInv);
        // orgObj 已在帧开始处算好

        // 与包围球求交（物体空间，球心在原点）
        float b = vdot(orgObj, dirObj);
        float c = vdot(orgObj, orgObj) - bound * bound;
        float disc = b*b - c;
        if (disc < 0) continue;
        float sq = sqrtf(disc);
        float t0 = -b - sq;
        if (t0 < 0) t0 = 0;
        float t1 = -b + sq;

        statPixels++;

        // ---- 步进（物体空间，无旋转）
        float t = t0;
        bool hit = false;
        int steps = 0;
        for (int s = 0; s < p.maxSteps; s++) {
          V3 pos = vadd(orgObj, vmul(dirObj, t));
          float d = sdfBox(pos);
          steps++;
          if (d < p.hitEps) { hit = true; break; }
          t += d * p.stepScale;
          if (t > t1) break;
        }
        statSteps += steps;
        if (!hit) continue;
        statHits++;

        V3 lp = vadd(orgObj, vmul(dirObj, t));

        // ---- 法线
        V3 nObj = sdfNormal(lp);

        // ---- 判定在哪个面（主导轴）
        float ax = fabsf(nObj.x), ay = fabsf(nObj.y), az = fabsf(nObj.z);
        int fi; float sgn;
        if (ax >= ay && ax >= az) { sgn = nObj.x >= 0 ? 1.0f : -1.0f; fi = sgn > 0 ? 2 : 3; }
        else if (ay >= ax && ay >= az) { sgn = nObj.y >= 0 ? 1.0f : -1.0f; fi = sgn > 0 ? 4 : 5; }
        else { sgn = nObj.z >= 0 ? 1.0f : -1.0f; fi = sgn > 0 ? 0 : 1; }

        const FaceAxis &fa = FACES[fi];
        float pu = vdot(lp, fa.r);
        float pv = vdot(lp, fa.u);

        V3 nFinal = nObj;
        bool inPip = false;
        float openness = 1.0f, tEdge = 1.0f;
        V3 pipN;
        if (pipHit(fa, sgn, pu, pv, fa.val, pipN, openness, tEdge)) {
          inPip = true;
          nFinal = pipN;
        }

        // ---- 着色（世界空间光照）
        V3 nW = qrot(nFinal, q);
        V3 L = v3(p.lx, p.ly, p.lz);
        int r, g, b2;

        if (inPip) {
          // 坑内白漆：要「看得出是凹坑」，所以让法线倾斜参与明暗
          //   坑心 nz≈1（法线朝外）→ 最亮
          //   坑壁 nz→0（法线几乎平行面）→ 变暗
          //   再叠一点光照方向，但下限保持在 0.45 以上，任何一面都还是白的
          float lam = fmaxf(0.0f, vdot(nW, L));
          float wall = openness;                     // 1=坑心, 0=坑壁
          float k = 0.45f + 0.32f * wall + 0.23f * lam;
          if (k > 1.0f) k = 1.0f;
          // 坑沿一圈轻微压暗，强化「洞口」的边界
          if (tEdge > 0.86f) k *= 0.80f;
          r  = (int)(p.pipR * k); g  = (int)(p.pipG * k); b2 = (int)(p.pipB * k);
        } else {
          // AO：沿法线取 4 个距离样本
          float occ = 0.0f, scale = 1.0f;
          for (int s = 0; s < 4; s++) {
            float hd = 0.008f + 0.055f * s;
            V3 p2 = vadd(lp, vmul(nFinal, hd));
            float dd = sdfBox(p2);
            statSdf++;
            occ += (hd - dd) * scale;
            scale *= 0.72f;
          }
          float ao = 1.0f - fmaxf(0.0f, fminf(1.0f, occ * 7.0f));
          ao = 0.30f + 0.70f * ao;

          float lam = fmaxf(0.0f, vdot(nW, L));
          float hemi = 0.5f + 0.5f * nW.y;
          float k = 0.42f + 0.22f * hemi + 0.36f * lam;
          k *= ao;
          if (k > 1.02f) k = 1.02f;
          r  = (int)(p.bodyR * k);
          g  = (int)(p.bodyG * k);
          b2 = (int)(p.bodyB * k);
        }
        buf[(size_t)y * W + x] = rgb565(r, g, b2);
      }
    }
  }

private:
  int shX0 = 1, shY0 = 1, shX1 = 0, shY1 = 0;   // 阴影范围（空=1,1,0,0）

  // 落地软阴影（画在骰子之前，会被骰子覆盖）
  void drawGroundShadow(Quat q, float yOff, float focal) {
    // 先把范围置空：任何提前返回都不能留下上一帧的旧范围
    // （codex 审查发现：否则 bbox 会并入过期范围，推屏多推一块）
    shX0 = 1; shY0 = 1; shX1 = 0; shY1 = 0;

    const float gy = -0.62f;
    const float wz = p.camZ;
    if (wz <= 0.2f) return;
    const float scx = W * 0.5f;
    const float scy = H * 0.5f - focal * gy / wz;

    float rx = 0, rz = 0;
    for (int i = 0; i < 8; i++) {
      float sx = (i & 1) ? p.H : -p.H;
      float sy = (i & 2) ? p.H : -p.H;
      float sz = (i & 4) ? p.H : -p.H;
      V3 v = qrot(v3(sx, sy, sz), q);
      float ax = fabsf(v.x), az = fabsf(v.z);
      if (ax > rx) rx = ax;
      if (az > rz) rz = az;
    }
    const float sx = focal * (rx * 1.28f) / wz;
    const float sy = focal * (rz * 1.28f * 0.40f) / wz;
    if (sx < 1.0f || sy < 1.0f) return;   // 范围已在函数开头置空

    const float lift = 0.30f * fmaxf(0.0f, yOff);
    const float strength = 0.60f - lift;
    if (strength <= 0) return;   // 范围已在函数开头置空

    int x0 = (int)floorf(scx - sx), x1 = (int)ceilf(scx + sx);
    int y0 = (int)floorf(scy - sy), y1 = (int)ceilf(scy + sy);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > W - 1) x1 = W - 1;
    if (y1 > H - 1) y1 = H - 1;

    shX0 = x0; shY0 = y0; shX1 = x1; shY1 = y1;
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) {
        float ddx = (x + 0.5f - scx) / sx;
        float ddy = (y + 0.5f - scy) / sy;
        float d = sqrtf(ddx*ddx + ddy*ddy);
        if (d >= 1.0f) continue;
        float a = powf(1.0f - d, 2.4f) * strength;
        if (a <= 0) continue;
        // 往黑混合
        uint16_t c = buf[(size_t)y * W + x];
        int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
        r = (int)(r * (1.0f - a)); g = (int)(g * (1.0f - a)); b = (int)(b * (1.0f - a));
        buf[(size_t)y * W + x] = (uint16_t)((r << 11) | (g << 5) | b);
      }
    }
  }
};

// ---------------------------------------------------------------- 目标姿态
// 点数看的是「朝上」那一面。align 到 +Y，再用 TILT 把顶面转向相机。
static inline Quat targetForValue(int v, float rollAngle,
                                  float tiltX = -0.50f, float tiltY = 0.55f) {
  int fi = 0;
  for (int i = 0; i < 6; i++) if (FACES[i].val == v) { fi = i; break; }
  Quat align = qfromto(FACES[fi].n, v3(0, 1, 0));
  Quat roll  = qaxis(v3(0, 1, 0), rollAngle);
  Quat tilt  = qmul(qaxis(v3(1, 0, 0), tiltX), qaxis(v3(0, 1, 0), tiltY));
  return qnorm(qmul(tilt, qmul(roll, align)));
}

} // namespace dice
