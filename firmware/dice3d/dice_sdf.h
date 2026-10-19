// ============================================================================
//  dice_sdf.h — SDF ray-marching renderer core for the die (pure C++, no Arduino dependency)
//
//  Uses the same math as preview/index.html, with three key optimizations:
//
//  1) Transform each ray into object space only once.
//     The JavaScript version rotates the quaternion at every step (56 rotations per pixel at 56 steps).
//     Here the ray origin and direction are each rotated once, then marching stays in object space — 55 rotations saved.
//
//  2) The object-space bounding sphere has a fixed radius of H*sqrt(3), so it needs no per-frame calculation.
//
//  3) Output RGB565 for direct display use instead of RGBA32.
//
//  It can be compiled on macOS (see host_test.cpp) to validate rendering changes offline
//  for correctness and timing before flashing the board.
// ============================================================================
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

namespace dice {

// ---------------------------------------------------------------- Parameters
struct Params {
  // Geometry (die edge length = 1.0)
  float H         = 0.5f;     // half edge length
  float R         = 0.088f;   // corner radius (about 1.4 mm for a 16 mm die)
  float pipOff    = 0.29f;    // pip offset within a face
  float pipRad    = 0.098f;   // pip radius
  float pipDepth  = 0.34f;    // pit depth / pip radius

  // Camera
  float focal     = 1100.0f;  // focal length (based on a 480-pixel canvas)
  float camZ      = 4.2f;     // camera distance

  // Marching
  int   maxSteps  = 56;
  float hitEps    = 0.0008f;
  float stepScale = 0.92f;    // <1 is more stable but slower

  // Colors (0–255)
  uint8_t bodyR = 214, bodyG = 38, bodyB = 38;      // red body
  uint8_t pipR  = 252, pipG  = 250, pipB  = 246;    // white pips

  // Light direction (from the surface toward the source, normalized)
  float lx = -0.38f, ly = 0.70f, lz = -0.60f;
};

// ---------------------------------------------------------------- Basic math
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
/** Shortest arc that rotates unit vector a to b */
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

// ---------------------------------------------------------------- Six faces
// Opposite faces sum to 7. Standard Western handedness: 1 faces forward (+Z), 2 faces up (+Y), 4 is right (+X), and 3 is left (-X).
struct FaceAxis { V3 n, r, u; uint8_t val; };
static const FaceAxis FACES[6] = {
  { { 0, 0, 1}, { 1, 0, 0}, {0, 1, 0}, 1 },  // +Z
  { { 0, 0,-1}, {-1, 0, 0}, {0, 1, 0}, 6 },  // -Z
  { { 1, 0, 0}, { 0, 0,-1}, {0, 1, 0}, 4 },  // +X
  { {-1, 0, 0}, { 0, 0, 1}, {0, 1, 0}, 3 },  // -X
  { { 0, 1, 0}, { 1, 0, 0}, {0, 0,-1}, 2 },  // +Y
  { { 0,-1, 0}, { 1, 0, 0}, {0, 0, 1}, 5 },  // -Y
};

// 2D face coordinates for each pip value (up to 6)
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

// ---------------------------------------------------------------- Renderer
class Renderer {
public:
  Params p;
  Quat   qInv;              // world to object
  float  centerY = 0;
  int    W = 0, H = 0;
  uint16_t *buf = nullptr;

  // This frame’s die bounding box in render pixels, clipped to the canvas
  int bboxX0 = 0, bboxY0 = 0, bboxX1 = 0, bboxY1 = 0;

  // Performance statistics
  uint32_t statPixels = 0;   // pixels actually marched
  uint32_t statHits   = 0;   // hit pixels
  uint32_t statSteps  = 0;   // total march steps
  uint32_t statSdf    = 0;   // total SDF evaluations

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

  // Tetrahedral-gradient normal
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

  // Pip indentation test: returns true on a hit and provides the pit normal (object space) and rim factor
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

  /** Render one frame to buf (W×H, RGB565). q is the die orientation; yOff is the throw height. */
  void render(Quat q, float yOff) {
    statPixels = statHits = statSteps = statSdf = 0;

    qInv = qconj(q);
    centerY = yOff;

    // Die center in world space (0, yOff, camZ)
    const float cx = 0, cy = yOff, cz = p.camZ;
    // The die is centered at the object-space origin; bounding-sphere radius = H*sqrt(3).
    const float bound = p.H * 1.7320508f;

    const float halfW = W * 0.5f, halfH = H * 0.5f;
    // Scale focal length to the canvas; p.focal is based on a 480-pixel canvas.
    const float focal = p.focal * (W / 480.0f);

    // ---- Screen bounds: project 8 corners to skip empty pixels.
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

    // Clear only the bounding box instead of memset over the full screen.
    bboxX0 = bx0; bboxY0 = by0; bboxX1 = bx1; bboxY1 = by1;
    if (bx0 <= bx1 && by0 <= by1) {
      for (int yy = by0; yy <= by1; yy++)
        memset(buf + (size_t)yy * W + bx0, 0, (size_t)(bx1 - bx0 + 1) * 2);
    }

    // Compute the camera origin in object space once per frame instead of once per pixel.
    const V3 orgObj = qrot(v3(-cx, -cy, -cz), qInv);

    // Draw the shadow first (it may extend beyond the die bounds), then include it in the bounding box.
    drawGroundShadow(q, yOff, focal);
    if (shX0 <= shX1 && shY0 <= shY1) {
      if (shX0 < bboxX0) bboxX0 = shX0;
      if (shY0 < bboxY0) bboxY0 = shY0;
      if (shX1 > bboxX1) bboxX1 = shX1;
      if (shY1 > bboxY1) bboxY1 = shY1;
    }

    // ---- Per-pixel rendering
    for (int y = by0; y <= by1; y++) {
      float syv = halfH - (y + 0.5f);
      for (int x = bx0; x <= bx1; x++) {
        float sxv = (x + 0.5f) - halfW;

        // World-space ray direction (camera at the origin, looking toward +Z).
        float dx = sxv / focal, dy = syv / focal, dz = 1.0f;
        float dl = sqrtf(dx*dx + dy*dy + dz*dz);
        dx /= dl; dy /= dl; dz /= dl;

        // Transform the ray direction into object space; it varies per pixel.
        V3 dirObj = qrot(v3(dx, dy, dz), qInv);
        // orgObj was computed at the start of the frame.

        // Intersect the bounding sphere in object space (centered at the origin).
        float b = vdot(orgObj, dirObj);
        float c = vdot(orgObj, orgObj) - bound * bound;
        float disc = b*b - c;
        if (disc < 0) continue;
        float sq = sqrtf(disc);
        float t0 = -b - sq;
        if (t0 < 0) t0 = 0;
        float t1 = -b + sq;

        statPixels++;

        // ---- March in object space without further rotation.
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

        // ---- Normal
        V3 nObj = sdfNormal(lp);

        // ---- Determine the face from the dominant axis.
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

        // ---- Shading with world-space lighting
        V3 nW = qrot(nFinal, q);
        V3 L = v3(p.lx, p.ly, p.lz);
        int r, g, b2;

        if (inPip) {
          // White paint inside each pip: shade the sloped normal so the indentation reads clearly.
          //   Pit center nz≈1 (normal points outward) → brightest.
          //   Pit wall nz→0 (normal nearly parallel to the face) → darker.
          //   Blend in the light direction while keeping the value above 0.45 so every face remains white.
          float lam = fmaxf(0.0f, vdot(nW, L));
          float wall = openness;                     // 1 = pit center, 0 = pit wall
          float k = 0.45f + 0.32f * wall + 0.23f * lam;
          if (k > 1.0f) k = 1.0f;
          // Slightly darken the rim to define the pip edge.
          if (tEdge > 0.86f) k *= 0.80f;
          r  = (int)(p.pipR * k); g  = (int)(p.pipG * k); b2 = (int)(p.pipB * k);
        } else {
          // AO: take four distance samples along the normal.
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
  int shX0 = 1, shY0 = 1, shX1 = 0, shY1 = 0;   // shadow bounds (empty = 1,1,0,0)

  // Soft ground shadow (drawn before the die, which can cover it).
  void drawGroundShadow(Quat q, float yOff, float focal) {
    // Clear the bounds first so early returns cannot leave stale values from the previous frame.
    // (Review found stale bounds could enlarge the merged box and cause an extra display transfer.)
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
    if (sx < 1.0f || sy < 1.0f) return;   // Bounds were cleared at the start of the function.

    const float lift = 0.30f * fmaxf(0.0f, yOff);
    const float strength = 0.60f - lift;
    if (strength <= 0) return;   // Bounds were cleared at the start of the function.

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
        // Blend toward black.
        uint16_t c = buf[(size_t)y * W + x];
        int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
        r = (int)(r * (1.0f - a)); g = (int)(g * (1.0f - a)); b = (int)(b * (1.0f - a));
        buf[(size_t)y * W + x] = (uint16_t)((r << 11) | (g << 5) | b);
      }
    }
  }
};

// ---------------------------------------------------------------- Target orientation
// The result is the face pointing up. Align it to +Y, then use TILT to turn the top face toward the camera.
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
