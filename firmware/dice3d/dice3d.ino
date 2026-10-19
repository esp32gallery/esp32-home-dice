/*
 * 3D Dice — Waveshare ESP32-S3-Touch-AMOLED-2.16 (CO5300 480x480 + CST9220 + QMI8658)
 *
 * Software-only 3D:quaternion orientation → perspective projection → convex polygon scanline rasterization → 2× display scaling
 * renders internally at 240×240 and scales dirty rectangles to 480×480, transferring only changed areas (CO5300 requires even alignment)
 *
 * Roll input: touch, shake gesture, or onboard GPIO18 button
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "Arduino_GFX_Library.h"
#include <Wire.h>
#include <SensorQMI8658.hpp>
#include <TouchDrvCSTXXX.hpp>
#include "HWCDC.h"
#include "dice_sdf.h"

// ---------------------------------------------------------------- Renderer selection
//   0 = legacy 6-face scanline renderer (lower CPU use, basic quality)
//   1 = SDF ray marching (higher quality, higher CPU use)
// Override with -DDICE_USE_SDF=1.
#ifndef DICE_USE_SDF
#define DICE_USE_SDF 1
#endif

// Internal SDF render resolution (1:1 display, no scaling)
#ifndef SDF_RES
#define SDF_RES 240
#endif

// ------------------------------------------------------------------ Pins
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK  38
#define LCD_RESET 39
#define LCD_CS    12
#define IIC_SDA   15
#define IIC_SCL   14
#define TP_INT    11
#define TP_RST    40
#define KEY_PIN   18

// ------------------------------------------------------------------ Render parameters
#define SCR_W  480
#define SCR_H  480
#define LOW_W  240
#define LOW_H  240
#define SCALE  2

#define FOCAL  400.0f
#define CAM_Z  4.7f

// Slight resting tilt to show the front, top, and side faces for depth.
#define TILT_X (-0.78f)   // Tilt forward to turn the upward-facing side toward the camera.
#define TILT_Y (0.55f)

// Die size: edge length 1.0 (-0.5 .. +0.5)
#define HALF     0.5f
#define PIP_OFF  0.29f
#define PIP_RAD  0.082f

// Animation duration
// ---------------------------------------------------------------- Debug logging
// IMPORTANT: USB CDC printf can block when nothing is reading the serial port.
//    When the buffer fills, Serial.printf can stall and freeze the UI (measured FPS dropped from 30 to 0.5).
//    Runtime logging is disabled by default. Enable it only when needed and always use SAFE_LOG below
//    (checks availableForWrite first; drops output when the buffer is full, never blocks).
#define DICE_DEBUG_LOG 1
// Diagnostics: roll once two seconds after startup and log each frame.
#define DICE_SELFTEST 0

// Use a macro instead of a function: Arduino treats it as the first function in the file and generates
// a prototype before struct V3, causing “V3 does not name a type” during compilation.
#if DICE_DEBUG_LOG
  #define SAFE_LOG(...) do { \
      if (USBSerial.availableForWrite() >= 96) USBSerial.printf(__VA_ARGS__); \
    } while (0)
#else
  #define SAFE_LOG(...) do {} while (0)
#endif

#define SPIN_MS    780
#define ROLL_MS   1150

// ------------------------------------------------------------------ Base vectors and quaternions
struct V3 { float x, y, z; };
struct Quat { float w, x, y, z; };

static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 vmul(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float vdot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 vcross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline float vlen(V3 a) { return sqrtf(vdot(a, a)); }
static inline V3 vnorm(V3 a) {
  float l = vlen(a);
  return (l > 1e-6f) ? vmul(a, 1.0f / l) : v3(0, 0, 0);
}

static inline Quat qmul(Quat a, Quat b) {
  Quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}
static inline Quat qnorm(Quat q) {
  float l = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (l < 1e-8f) { Quat i = {1, 0, 0, 0}; return i; }
  float s = 1.0f / l;
  Quat r = {q.w * s, q.x * s, q.y * s, q.z * s};
  return r;
}
static inline Quat qaxis(V3 axis, float ang) {
  V3 a = vnorm(axis);
  float h = ang * 0.5f, s = sinf(h);
  Quat r = {cosf(h), a.x * s, a.y * s, a.z * s};
  return r;
}
// Shortest arc that rotates a to b
static Quat qfromto(V3 a, V3 b) {
  a = vnorm(a); b = vnorm(b);
  float d = vdot(a, b);
  if (d > 0.99999f) { Quat i = {1, 0, 0, 0}; return i; }
  if (d < -0.99999f) {
    V3 ax = vcross(a, v3(1, 0, 0));
    if (vlen(ax) < 1e-4f) ax = vcross(a, v3(0, 1, 0));
    ax = vnorm(ax);
    Quat r = {0, ax.x, ax.y, ax.z};
    return r;
  }
  V3 ax = vcross(a, b);
  Quat r = {1.0f + d, ax.x, ax.y, ax.z};
  return qnorm(r);
}
static inline V3 qrot(V3 v, Quat q) {
  // v' = v + 2w(q.xyz x v) + 2 q.xyz x (q.xyz x v)
  V3 u = v3(q.x, q.y, q.z);
  V3 t = vmul(vcross(u, v), 2.0f);
  return vadd(vadd(v, vmul(t, q.w)), vcross(u, t));
}
static Quat qslerp(Quat a, Quat b, float t) {
  float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < 0.0f) { b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z; d = -d; }
  if (d > 0.9995f) {
    Quat r = {a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
              a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    return qnorm(r);
  }
  float th = acosf(d);
  float s = sinf(th);
  float s0 = sinf((1.0f - t) * th) / s;
  float s1 = sinf(t * th) / s;
  Quat r = {a.w * s0 + b.w * s1, a.x * s0 + b.x * s1,
            a.y * s0 + b.y * s1, a.z * s0 + b.z * s1};
  return qnorm(r);
}

// ------------------------------------------------------------------ Die geometry
struct Face {
  V3 n;        // outward normal
  V3 c;        // face center
  V3 r;        // face-local right
  V3 u;        // face-local up
  uint8_t val; // pip value
};

// Opposite face values sum to 7：+Z=1, -Z=6, +Y=2, -Y=5, +X=3, -X=4
// Standard Western die handedness (right-handed / counter-clockwise): with 1 forward (+Z) and 2 up (+Y), 4 is right (+X) and 3 is left (-X).
//   Mnemonic: “1 up, 2 left, 3 right”; viewed from the 1-2-3 corner, 1→2→3 is counter-clockwise.
//   Opposite faces always sum to 7: 1-6 / 2-5 / 3-4.
//   Earlier versions mirrored +X=3 / -X=4 (clockwise); corrected to match StandardDice.pdf.
//   Pip layout: 2/3 use diagonals, 4/5 use four corners (5 also has a center pip), and 6 uses two columns of three.
static const Face FACES[6] = {
  { { 0, 0, 1}, { 0, 0, HALF}, { 1, 0, 0}, {0, 1, 0}, 1 },  // +Z forward
  { { 0, 0,-1}, { 0, 0,-HALF}, {-1, 0, 0}, {0, 1, 0}, 6 },  // -Z rear
  { { 1, 0, 0}, { HALF, 0, 0}, { 0, 0,-1}, {0, 1, 0}, 4 },  // +X right
  { {-1, 0, 0}, {-HALF, 0, 0}, { 0, 0, 1}, {0, 1, 0}, 3 },  // -X left
  { { 0, 1, 0}, { 0, HALF, 0}, { 1, 0, 0}, {0, 0,-1}, 2 },  // +Y up
  { { 0,-1, 0}, { 0,-HALF, 0}, { 1, 0, 0}, {0, 0, 1}, 5 },  // -Y down
};


// Pip value to 2D face coordinates (relative to the cube edge length).
// Up to six pips.
static const float PIPS[7][6][2] = {
  {},                                                                  // 0
  {{0, 0}},                                                            // 1
  {{-PIP_OFF, -PIP_OFF}, {PIP_OFF, PIP_OFF}},                          // 2
  {{-PIP_OFF, -PIP_OFF}, {0, 0}, {PIP_OFF, PIP_OFF}},                  // 3
  {{-PIP_OFF, -PIP_OFF}, {PIP_OFF, -PIP_OFF},
   {-PIP_OFF, PIP_OFF}, {PIP_OFF, PIP_OFF}},                           // 4
  {{-PIP_OFF, -PIP_OFF}, {PIP_OFF, -PIP_OFF}, {0, 0},
   {-PIP_OFF, PIP_OFF}, {PIP_OFF, PIP_OFF}},                           // 5
  {{-PIP_OFF, -PIP_OFF}, {PIP_OFF, -PIP_OFF},
   {-PIP_OFF, 0}, {PIP_OFF, 0},
   {-PIP_OFF, PIP_OFF}, {PIP_OFF, PIP_OFF}},                           // 6
};

// ------------------------------------------------------------------ Colors
static inline uint16_t rgb565(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static const uint16_t COL_BLACK = 0x0000;
static const uint16_t COL_PIP   = rgb565(252, 250, 246);   // white pips
// bright red body
static const int BASE_R = 214, BASE_G = 38, BASE_B = 38;

// Light direction (from the surface toward the source): upper-left and slightly forward.
static const V3 LIGHT = {-0.42f, 0.62f, -0.66f};

// ------------------------------------------------------------------ Globals
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, SCR_W, SCR_H, 0, 0, 0, 0);

static uint16_t *lowBuf = nullptr;  // LOW_W*LOW_H RGB565
static uint16_t *outBuf = nullptr;  // SCR_W*SCR_H RGB565

#if DICE_USE_SDF
// SDF renderer: render directly at SDF_RES×SDF_RES, then scale 2× for display.
static dice::Renderer sdfR;
static uint16_t *sdfBuf = nullptr;              // SDF_RES² RGB565
static int sdfW = SDF_RES, sdfH = SDF_RES;
static uint32_t sdfFrameMs = 0;                 // measured frame time
#endif

// Whether the frame needs redraw; skip rendering while idle to save CPU and power and keep input responsive.
static bool frameDirty = true;
#if DICE_USE_SDF
#endif

static int prevX0 = 0, prevY0 = 0, prevX1 = SCR_W - 1, prevY1 = SCR_H - 1;
static bool firstFrame = true;

HWCDC USBSerial;

// Sensors
static SensorQMI8658 qmi;
static IMUdata acc;
static bool imuOk = false;

static TouchDrvCST92xx touch;
static int16_t tx[2], ty[2];
static volatile bool touchPending = false;
static bool touchOk = false;

// ------------------------------------------------------------------ State
enum Mode { MODE_IDLE, MODE_ROLLING };
static Mode mode = MODE_IDLE;

static Quat qCur = {1, 0, 0, 0};
static Quat qTarget = {1, 0, 0, 0};
static Quat qSpinStart = {1, 0, 0, 0};
static Quat qSlerpFrom = {1, 0, 0, 0};
static V3 spinAxis = {1, 0, 0};
static float spinSpeed = 10.0f;
static uint32_t rollStartMs = 0;
static uint32_t lastInputMs = 0;
static uint32_t lastRollMs = 0;
static uint8_t lastValue = 0;
static float hopY = 0.0f;

// Brightness / burn-in prevention
static int curBrightness = 200;
static uint32_t lastBrightnessCheck = 0;

// Frame-rate statistics
static uint32_t fpsFrames = 0, fpsLastMs = 0;

// ------------------------------------------------------------------ Rasterization
static inline void fillRow(uint16_t *buf, int y, int xl, int xr, uint16_t c) {
  if (y < 0 || y >= LOW_H) return;
  if (xl < 0) xl = 0;
  if (xr >= LOW_W) xr = LOW_W - 1;
  if (xl > xr) return;
  uint16_t *p = buf + (size_t)y * LOW_W;
  for (int x = xl; x <= xr; x++) p[x] = c;
}

// Convex polygon scanline fill
static void fillConvex(const float *px, const float *py, int n, uint16_t color) {
  float fminY = py[0], fmaxY = py[0];
  for (int i = 1; i < n; i++) {
    if (py[i] < fminY) fminY = py[i];
    if (py[i] > fmaxY) fmaxY = py[i];
  }
  int y0 = (int)ceilf(fminY);
  int y1 = (int)floorf(fmaxY);
  if (y0 < 0) y0 = 0;
  if (y1 > LOW_H - 1) y1 = LOW_H - 1;
  if (y0 > y1) return;

  for (int y = y0; y <= y1; y++) {
    float yc = (float)y + 0.5f;
    float xl = 1e30f, xr = -1e30f;
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      float ay = py[i], by = py[j];
      if (ay == by) continue;
      float lo = ay < by ? ay : by;
      float hi = ay < by ? by : ay;
      if (yc < lo || yc >= hi) continue;
      float ax = px[i], bx = px[j];
      float t = (yc - ay) / (by - ay);
      float x = ax + t * (bx - ax);
      if (x < xl) xl = x;
      if (x > xr) xr = x;
    }
    if (xl <= xr) fillRow(lowBuf, y, (int)ceilf(xl), (int)floorf(xr), color);
  }
}

// Polygon x-span on a scanline
static void polySpan(const float *px, const float *py, int n, float yc, float *out) {
  float xl = 1e30f, xr = -1e30f;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    float ay = py[i], by = py[j];
    if (ay == by) continue;
    float lo = ay < by ? ay : by;
    float hi = ay < by ? by : ay;
    if (yc < lo || yc >= hi) continue;
    float t = (yc - ay) / (by - ay);
    float x = px[i] + t * (px[j] - px[i]);
    if (x < xl) xl = x;
    if (x > xr) xr = x;
  }
  out[0] = xl;
  out[1] = xr;
}

// Solid circle clipped to the face polygon so pips do not spill over at shallow viewing angles.
static void fillCircleClipped(int cx, int cy, int rad, uint16_t color,
                              const float *px, const float *py, int n) {
  if (rad < 1) rad = 1;
  int y0 = cy - rad, y1 = cy + rad;
  if (y0 < 0) y0 = 0;
  if (y1 > LOW_H - 1) y1 = LOW_H - 1;
  int r2 = rad * rad;
  for (int y = y0; y <= y1; y++) {
    int dy = y - cy;
    int rem = r2 - dy * dy;
    if (rem < 0) continue;
    int dx = (int)sqrtf((float)rem);
    int l = cx - dx, r = cx + dx;

    float span[2];
    polySpan(px, py, n, (float)y + 0.5f, span);
    if (span[0] > span[1]) continue;
    if ((float)l < span[0]) l = (int)ceilf(span[0]);
    if ((float)r > span[1]) r = (int)floorf(span[1]);
    fillRow(lowBuf, y, l, r, color);
  }
}

// ------------------------------------------------------------------ Render one frame
struct Proj { float x, y, z; bool ok; };

static void renderDice(Quat q, float yOff) {
  // 1) Clear the low-resolution buffer
  memset(lowBuf, 0, (size_t)LOW_W * LOW_H * 2);

  // 2) Transform and project vertices
  V3 verts[8];
  Proj proj[8];
  static const float CORN[8][3] = {
    {-HALF,-HALF,-HALF}, {HALF,-HALF,-HALF}, {HALF,HALF,-HALF}, {-HALF,HALF,-HALF},
    {-HALF,-HALF, HALF}, {HALF,-HALF, HALF}, {HALF,HALF, HALF}, {-HALF,HALF, HALF},
  };
  for (int i = 0; i < 8; i++) {
    V3 v = qrot(v3(CORN[i][0], CORN[i][1], CORN[i][2]), q);
    v.y += yOff;
    v.z += CAM_Z;
    verts[i] = v;
    if (v.z > 0.15f) {
      proj[i].x = LOW_W * 0.5f + FOCAL * v.x / v.z;
      proj[i].y = LOW_H * 0.5f - FOCAL * v.y / v.z;
      proj[i].z = v.z;
      proj[i].ok = true;
    } else {
      proj[i].ok = false;
    }
  }

  // 3) Collect visible faces
  struct VisFace { int idx; float depth; V3 n; };
  VisFace vis[6];
  int nvis = 0;
  for (int f = 0; f < 6; f++) {
    const Face &fc = FACES[f];
    V3 nr = qrot(fc.n, q);
    V3 cc = qrot(fc.c, q);
    cc.y += yOff;
    cc.z += CAM_Z;
    // Camera is at the origin and looks toward +Z; a normal faces the camera when dot(n, center) < 0.
    if (vdot(nr, cc) >= 0.0f) continue;
    vis[nvis].idx = f;
    vis[nvis].depth = cc.z;
    vis[nvis].n = nr;
    nvis++;
  }
  // Painter’s algorithm: far to near.
  for (int i = 1; i < nvis; i++) {
    VisFace t = vis[i];
    int j = i - 1;
    while (j >= 0 && vis[j].depth < t.depth) { vis[j + 1] = vis[j]; j--; }
    vis[j + 1] = t;
  }

  // 4) Draw each face
  for (int vi = 0; vi < nvis; vi++) {
    const Face &fc = FACES[vis[vi].idx];

    // The face’s four vertex indices, in faceCorner order.
    // Vertex indices for face f’s four corners, ordered as (-,-), (+,-), (+,+), (-,+).
    static const int FV[6][4] = {
      {4, 5, 6, 7},  // +Z
      {1, 0, 3, 2},  // -Z
      {5, 1, 2, 6},  // +X
      {0, 4, 7, 3},  // -X
      {7, 6, 2, 3},  // +Y
      {0, 1, 5, 4},  // -Y
    };
    const int *fv = FV[vis[vi].idx];
    bool ok = true;
    float px[4], py[4];
    for (int k = 0; k < 4; k++) {
      if (!proj[fv[k]].ok) { ok = false; break; }
      px[k] = proj[fv[k]].x;
      py[k] = proj[fv[k]].y;
    }
    if (!ok) continue;

    // Lighting
    float lam = vdot(vis[vi].n, vnorm(LIGHT));
    if (lam < 0.0f) lam = 0.0f;
    float inten = 0.38f + 0.62f * lam;
    if (inten > 1.0f) inten = 1.0f;
    uint16_t body = rgb565((int)(BASE_R * inten), (int)(BASE_G * inten), (int)(BASE_B * inten));

    fillConvex(px, py, 4, body);

    // pip value
    int val = fc.val;
    for (int p = 0; p < val; p++) {
      float a = PIPS[val][p][0];
      float b = PIPS[val][p][1];
      V3 pw = vadd(fc.c, vadd(vmul(fc.r, a), vmul(fc.u, b)));
      V3 pc = qrot(pw, q);
      pc.y += yOff;
      pc.z += CAM_Z;
      if (pc.z <= 0.15f) continue;
      float sx = LOW_W * 0.5f + FOCAL * pc.x / pc.z;
      float sy = LOW_H * 0.5f - FOCAL * pc.y / pc.z;
      float rr = FOCAL * PIP_RAD / pc.z;
      // Slightly shade pips with the lighting so they do not look flat black.
      uint16_t pc2 = COL_PIP;
      if (inten < 0.55f) pc2 = rgb565(52, 50, 56);
      fillCircleClipped((int)(sx + 0.5f), (int)(sy + 0.5f), (int)(rr + 0.5f), pc2, px, py, 4);
    }
  }
}

// Compute the die bounds in low-resolution space.
static void diceBBox(Quat q, float yOff, int *bx0, int *by0, int *bx1, int *by1) {
  float mnx = 1e30f, mny = 1e30f, mxx = -1e30f, mxy = -1e30f;
  static const float CORN[8][3] = {
    {-HALF,-HALF,-HALF}, {HALF,-HALF,-HALF}, {HALF,HALF,-HALF}, {-HALF,HALF,-HALF},
    {-HALF,-HALF, HALF}, {HALF,-HALF, HALF}, {HALF,HALF, HALF}, {-HALF,HALF, HALF},
  };
  for (int i = 0; i < 8; i++) {
    V3 v = qrot(v3(CORN[i][0], CORN[i][1], CORN[i][2]), q);
    v.y += yOff;
    v.z += CAM_Z;
    if (v.z <= 0.15f) continue;
    float sx = LOW_W * 0.5f + FOCAL * v.x / v.z;
    float sy = LOW_H * 0.5f - FOCAL * v.y / v.z;
    if (sx < mnx) mnx = sx;
    if (sx > mxx) mxx = sx;
    if (sy < mny) mny = sy;
    if (sy > mxy) mxy = sy;
  }
  if (mnx > mxx) { mnx = 0; mny = 0; mxx = LOW_W - 1; mxy = LOW_H - 1; }
  *bx0 = (int)floorf(mnx) - 2;
  *by0 = (int)floorf(mny) - 2;
  *bx1 = (int)ceilf(mxx) + 2;
  *by1 = (int)ceilf(mxy) + 2;
}

// Scale the low-resolution buffer rectangle 2× for display (dirty rectangle with even alignment).
static void blitRect(int lx0, int ly0, int lx1, int ly1) {
  if (lx0 < 0) lx0 = 0;
  if (ly0 < 0) ly0 = 0;
  if (lx1 > LOW_W - 1) lx1 = LOW_W - 1;
  if (ly1 > LOW_H - 1) ly1 = LOW_H - 1;
  if (lx0 > lx1 || ly0 > ly1) return;

  // Low-resolution to screen coordinates (even-aligned).
  int sx0 = lx0 * SCALE;
  int sy0 = ly0 * SCALE;
  int sx1 = (lx1 + 1) * SCALE - 1;
  int sy1 = (ly1 + 1) * SCALE - 1;
  if (sx0 & 1) sx0--;
  if (sy0 & 1) sy0--;
  if (!(sx1 & 1)) sx1++;
  if (!(sy1 & 1)) sy1++;
  if (sx0 < 0) sx0 = 0;
  if (sy0 < 0) sy0 = 0;
  if (sx1 > SCR_W - 1) sx1 = SCR_W - 1;
  if (sy1 > SCR_H - 1) sy1 = SCR_H - 1;

  int w = sx1 - sx0 + 1;
  int h = sy1 - sy0 + 1;
  if (w <= 0 || h <= 0) return;

  // Scale into outBuf.
  for (int y = 0; y < h; y += 2) {
    int sy = (sy0 + y) >> 1;
    const uint16_t *srow = lowBuf + (size_t)sy * LOW_W;
    uint16_t *d0 = outBuf + (size_t)y * w;
    uint16_t *d1 = d0 + w;
    for (int x = 0; x < w; x += 2) {
      uint16_t c = srow[(sx0 + x) >> 1];
      d0[x] = c; d0[x + 1] = c;
      d1[x] = c; d1[x + 1] = c;
    }
  }

  gfx->draw16bitRGBBitmap(sx0, sy0, outBuf, w, h);

  prevX0 = sx0; prevY0 = sy0; prevX1 = sx1; prevY1 = sy1;
}

#if DICE_USE_SDF
// SDF path: render to sdfBuf, then scale the full region 2× for display (CO5300 requires even alignment).
static void drawFrameSDF() {
  uint32_t t0 = micros();

  // Both the firmware and SDF core use the same Quat type, so it can be passed directly.
  dice::Quat q;
  q.w = qCur.w; q.x = qCur.x; q.y = qCur.y; q.z = qCur.z;

  sdfR.buf = sdfBuf;
  sdfR.W = sdfW;
  sdfR.H = sdfH;
  sdfR.render(q, hopY);

  sdfFrameMs = (micros() - t0) / 1000;

  // Transfer only the die bounds, not the full screen — a full 480² frame takes 23 ms at 40 MHz QSPI,
  //   while the die occupies about 75% of each dimension (~56% of the area), saving nearly half the transfer time.
  // CO5300 requires even start coordinates, width, and height, so round bounds down to even values.
  int lx0 = sdfR.bboxX0, ly0 = sdfR.bboxY0;
  int lx1 = sdfR.bboxX1, ly1 = sdfR.bboxY1;
  if (lx0 < 0) lx0 = 0;
  if (ly0 < 0) ly0 = 0;
  if (lx1 > sdfW - 1) lx1 = sdfW - 1;
  if (ly1 > sdfH - 1) ly1 = sdfH - 1;
  if (lx0 > lx1 || ly0 > ly1) return;

  int sx0 = lx0 * 2, sy0 = ly0 * 2;
  int sx1 = (lx1 + 1) * 2 - 1, sy1 = (ly1 + 1) * 2 - 1;
  if (sx0 & 1) sx0--;
  if (sy0 & 1) sy0--;
  if (!(sx1 & 1)) sx1++;
  if (!(sy1 & 1)) sy1++;
  if (sx0 < 0) sx0 = 0;
  if (sy0 < 0) sy0 = 0;
  if (sx1 > SCR_W - 1) sx1 = SCR_W - 1;
  if (sy1 > SCR_H - 1) sy1 = SCR_H - 1;

  int w = sx1 - sx0 + 1, h = sy1 - sy0 + 1;
  if (w <= 0 || h <= 0) return;

  // 2× Scale into outBuf (this region only).
  for (int y = 0; y < h; y += 2) {
    int ly = (sy0 + y) >> 1;
    const uint16_t *src = sdfBuf + (size_t)ly * sdfW;
    uint16_t *d0 = outBuf + (size_t)y * w;
    uint16_t *d1 = d0 + w;
    for (int x = 0; x < w; x += 2) {
      uint16_t c = src[(sx0 + x) >> 1];
      d0[x] = c; d0[x+1] = c;
      d1[x] = c; d1[x+1] = c;
    }
  }
  gfx->draw16bitRGBBitmap(sx0, sy0, outBuf, w, h);
}
#endif

static void drawFrame() {
  if (!frameDirty) return;      // Frame unchanged; skip redraw.
  frameDirty = false;

#if DICE_USE_SDF
  drawFrameSDF();
  return;
#endif
  int bx0, by0, bx1, by1;
  diceBBox(qCur, hopY, &bx0, &by0, &bx1, &by1);

  int ux0 = bx0, uy0 = by0, ux1 = bx1, uy1 = by1;
  if (!firstFrame) {
    if (prevX0 / SCALE < ux0) ux0 = prevX0 / SCALE;
    if (prevY0 / SCALE < uy0) uy0 = prevY0 / SCALE;
    if (prevX1 / SCALE > ux1) ux1 = prevX1 / SCALE;
    if (prevY1 / SCALE > uy1) uy1 = prevY1 / SCALE;
  }
  firstFrame = false;

  renderDice(qCur, hopY);
  blitRect(ux0, uy0, ux1, uy1);
}

// ------------------------------------------------------------------ Roll the die
static Quat targetForValue(int v, float rollAngle) {
  int fi = -1;
  for (int i = 0; i < 6; i++) if (FACES[i].val == v) { fi = i; break; }
  if (fi < 0) fi = 0;
  // For a real die, the result is the face pointing up, not the face pointing toward the camera.
  Quat align = qfromto(FACES[fi].n, v3(0, 1, 0));
  Quat roll = qaxis(v3(0, 1, 0), rollAngle);
  Quat tilt = qmul(qaxis(v3(1, 0, 0), TILT_X), qaxis(v3(0, 1, 0), TILT_Y));
  return qnorm(qmul(tilt, qmul(roll, align)));
}

static void startRoll() {
  if (mode == MODE_ROLLING) return;

  int v = 1 + (esp_random() % 6);
  lastValue = v;

  // Randomize the spin axis so it does not align with a coordinate axis and appear motionless.
  V3 ax = v3(((int)(esp_random() % 2001) - 1000) / 1000.0f,
             ((int)(esp_random() % 2001) - 1000) / 1000.0f,
             ((int)(esp_random() % 2001) - 1000) / 1000.0f);
  if (vlen(ax) < 0.2f) ax = v3(1, 0.4f, 0.2f);
  spinAxis = vnorm(ax);
  spinSpeed = 11.0f + (esp_random() % 700) / 100.0f;  // 11 ~ 18 rad/s

  // Allow only a small in-plane rotation when settling so the pips stay upright.
  float rollAngle = (((int)(esp_random() % 300)) - 150) / 1000.0f;  // -0.15 ~ +0.15
  qTarget = targetForValue(v, rollAngle);

  qSpinStart = qCur;
  qSlerpFrom = qCur;
  mode = MODE_ROLLING;
  frameDirty = true;
  rollStartMs = millis();
  lastInputMs = millis();
  lastRollMs = millis();

  SAFE_LOG("[dice] roll -> %d\n", v);
}

static void updateAnimation(uint32_t now) {
  if (mode != MODE_ROLLING) {
    qCur = qTarget;
    hopY = 0.0f;
    return;
  }

  uint32_t el = now - rollStartMs;
  SAFE_LOG("[frame] el=%u q=(%.3f,%.3f,%.3f,%.3f) hop=%.3f\n",
           el, qCur.w, qCur.x, qCur.y, qCur.z, hopY);

  if (el < SPIN_MS) {
    // Free tumble
    static uint32_t lastMs = 0;
    if (lastMs == 0) lastMs = rollStartMs;
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    if (dt > 0.05f) dt = 0.05f;
    qCur = qnorm(qmul(qCur, qaxis(spinAxis, spinSpeed * dt)));
    frameDirty = true;

    // Lift
    float t = (float)el / (float)SPIN_MS;
    hopY = 0.42f * sinf(t * (float)M_PI);
    qSlerpFrom = qCur;
  } else if (el < ROLL_MS) {
    // Ease into the resting pose
    float t = (float)(el - SPIN_MS) / (float)(ROLL_MS - SPIN_MS);
    // easeOutCubic
    float e = 1.0f - powf(1.0f - t, 3.0f);
    qCur = qslerp(qSlerpFrom, qTarget, e);
    hopY = 0.0f;
    frameDirty = true;
  } else {
    qCur = qTarget;
    hopY = 0.0f;
    frameDirty = true;
    mode = MODE_IDLE;
    SAFE_LOG("[dice] landed %d\n", lastValue);
  }
}

// ------------------------------------------------------------------ Input
void IRAM_ATTR onTouchISR() { touchPending = true; }

static bool pollTouch() {
  if (!touchOk) return false;
  bool pending;
  noInterrupts();
  pending = touchPending;
  touchPending = false;
  interrupts();
  if (!pending) return false;

  uint8_t n = touch.getPoint(tx, ty, touch.getSupportTouchPoint());
  return n > 0;
}

static float shakeScore = 0.0f;
static bool pollShake() {
  if (!imuOk) return false;
  if (!qmi.getDataReady()) return false;
  if (!qmi.getAccelerometer(acc.x, acc.y, acc.z)) return false;

  float mag = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
  float dev = fabsf(mag - 1.0f);
  if (dev > 0.55f) shakeScore += 1.0f;
  else shakeScore *= 0.75f;
  if (shakeScore > 4.0f) {
    shakeScore = 0.0f;
    return true;
  }
  return false;
}

static int keyStable = HIGH;
static int keyRaw = HIGH;
static uint32_t keyChangeMs = 0;
static bool pollKey(uint32_t now) {
  int r = digitalRead(KEY_PIN);
  if (r != keyRaw) {
    keyRaw = r;
    keyChangeMs = now;
  }
  if (now - keyChangeMs > 25 && keyStable != keyRaw) {
    keyStable = keyRaw;
    if (keyStable == LOW) return true;
  }
  return false;
}

// ------------------------------------------------------------------ Brightness
static void updateBrightness(uint32_t now, bool input) {
  if (input) {
    lastInputMs = now;
    if (curBrightness != 200) {
      curBrightness = 200;
      gfx->setBrightness(curBrightness);
    }
    return;
  }
  if (now - lastBrightnessCheck < 1000) return;
  lastBrightnessCheck = now;

  uint32_t idle = now - lastInputMs;
  int want = 200;
  if (idle > 180000UL) want = 12;       // 3 minutes: very dim
  else if (idle > 45000UL) want = 55;   // 45 seconds: dim

  if (want != curBrightness) {
    curBrightness = want;
    gfx->setBrightness(curBrightness);
  }
}

#if DICE_USE_SDF
// Send 'b' through the USB monitor to run this on demand after the monitor opens.
static void runSdfBench() {
  static const int RES[] = {240, 200, 180, 160, 140, 120};
  USBSerial.println("=== SDF BENCH ===");
  USBSerial.println("res,ms,steps,sdf,hit");
  for (int r : RES) {
    uint16_t *tmp = (uint16_t *)heap_caps_malloc((size_t)r * r * 2, MALLOC_CAP_SPIRAM);
    if (!tmp) { USBSerial.printf("%d,alloc_fail\n", r); continue; }
    dice::Renderer bench;
    bench.W = r; bench.H = r; bench.buf = tmp;
    bench.p.focal = 1100.0f; bench.p.camZ = 4.2f;
    dice::Quat bq = dice::targetForValue(3, 0.0f);
    bench.render(bq, 0.0f);
    const int N = 5;
    uint32_t t0 = micros();
    for (int k = 0; k < N; k++) bench.render(bq, 0.0f);
    uint32_t ms = (micros() - t0) / 1000 / N;
    USBSerial.printf("%d,%u,%u,%u,%u\n", r, ms,
                     bench.statSteps, bench.statSdf, bench.statHits);
    free(tmp);
  }
  USBSerial.println("=== BENCH END ===");
}
#endif

// ------------------------------------------------------------------ setup
void setup() {
  USBSerial.begin(115200);
  delay(150);

  Wire.begin(IIC_SDA, IIC_SCL);

  pinMode(KEY_PIN, INPUT_PULLUP);

  // IMPORTANT: follow the initialization order in the official 05_LVGL_Widgets example: touch → IMU → gfx->begin().
  //    Previously, initializing gfx->begin() before touch caused “touch not found” when running on battery.
  // Touch
  touch.setPins(TP_RST, TP_INT);
  touchOk = touch.begin(Wire, CST92XX_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (touchOk) {
    touch.setMaxCoordinates(SCR_W, SCR_H);
    touch.setSwapXY(true);
    touch.setMirrorXY(true, false);
    pinMode(TP_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TP_INT), onTouchISR, FALLING);
    USBSerial.printf("touch ok: %s\n", touch.getModelName());
  } else {
    USBSerial.println("touch not found (ignored)");
  }

  // IMU
  imuOk = qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (imuOk) {
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                            SensorQMI8658::ACC_ODR_500Hz,
                            SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();
    USBSerial.println("imu ok");
  } else {
    USBSerial.println("imu not found (ignored)");
  }

  // Display
  if (!gfx->begin()) {
    USBSerial.println("gfx->begin() failed");
    while (true) delay(1000);
  }
  bus->writeC8D8(0x36, 0xA0);
  gfx->fillScreen(COL_BLACK);
  gfx->setBrightness(200);

  // Buffers (PSRAM)
  lowBuf = (uint16_t *)heap_caps_malloc((size_t)LOW_W * LOW_H * 2, MALLOC_CAP_SPIRAM);
  outBuf = (uint16_t *)heap_caps_malloc((size_t)SCR_W * SCR_H * 2, MALLOC_CAP_SPIRAM);
  if (!lowBuf || !outBuf) {
    USBSerial.println("PSRAM alloc failed");
    gfx->fillScreen(COL_BLACK);
    while (true) delay(1000);
  }
  memset(lowBuf, 0, (size_t)LOW_W * LOW_H * 2);

#if DICE_USE_SDF
  sdfBuf = (uint16_t *)heap_caps_malloc((size_t)sdfW * sdfH * 2, MALLOC_CAP_SPIRAM);
  if (!sdfBuf) { USBSerial.println("SDF buffer allocation failed"); while (true) delay(1000); }
  sdfR.buf = sdfBuf; sdfR.W = sdfW; sdfR.H = sdfH;
  sdfR.p.focal = 1100.0f; sdfR.p.camZ = 4.2f;
#endif



  // Initial orientation: 1 faces forward.
  qTarget = targetForValue(1, 0.35f);
  qCur = qTarget;
  lastInputMs = millis();

  USBSerial.println("=== dice3d ready ===");
  drawFrame();
}

// ------------------------------------------------------------------ loop
void loop() {
  uint32_t now = millis();
#if DICE_USE_SDF
  if (USBSerial.available() && USBSerial.read() == 'b') runSdfBench();
#endif
  bool input = false;

#if DICE_SELFTEST
  static bool stDone = false;
  static uint32_t stRollAt = 0;
  if (!stDone) {
    if (stRollAt == 0) {
      stRollAt = now + 2000;           // 2 seconds after startup
      SAFE_LOG("[selftest] armed, will roll at +2000ms\n");
    } else if (now >= stRollAt && mode == MODE_IDLE) {
      SAFE_LOG("[selftest] >>> triggering roll\n");
      startRoll();
      stDone = true;
    }
  }
#endif

  if (pollTouch()) input = true;
  if (pollShake()) input = true;
  if (pollKey(now)) input = true;

  if (input) {
    lastInputMs = now;
    if (mode == MODE_IDLE && now - lastRollMs > 350) {
      startRoll();
    }
  }

  updateAnimation(now);
  drawFrame();
  updateBrightness(now, input);

  fpsFrames++;
  if (now - fpsLastMs >= 2000) {
#if DICE_USE_SDF
    SAFE_LOG("[fps] %.1f  sdf=%ums\n", fpsFrames * 1000.0f / (now - fpsLastMs), sdfFrameMs);
#else
    SAFE_LOG("[fps] %.1f\n", fpsFrames * 1000.0f / (now - fpsLastMs));
#endif
    fpsFrames = 0;
    fpsLastMs = now;
  }

  // Yield the CPU so USB CDC / IDLE tasks do not starve and worsen printf blocking.
  delay(1);
}
