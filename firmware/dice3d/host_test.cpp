// host_test.cpp —— 在 Mac 上跑 dice_sdf.h，验证正确性 + 量相对成本
//
// 编译： g++ -O2 -std=c++17 -o /tmp/dice_host host_test.cpp
// 运行： /tmp/dice_host [输出目录]
//
// 产出：
//   sdf_<res>_<val>.ppm  各点数落位后的渲染图（可直接看）
//   控制台打印每档分辨率的 ms/帧 与 SDF 求值次数
#include "dice_sdf.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>

using namespace dice;

static void writePPM(const char *path, const uint16_t *buf, int W, int H) {
  FILE *f = fopen(path, "wb");
  if (!f) { printf("  打不开 %s\n", path); return; }
  fprintf(f, "P6\n%d %d\n255\n", W, H);
  std::vector<uint8_t> row((size_t)W * 3);
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint16_t c = buf[(size_t)y * W + x];
      int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
      row[x*3+0] = (uint8_t)((r << 3) | (r >> 2));
      row[x*3+1] = (uint8_t)((g << 2) | (g >> 4));
      row[x*3+2] = (uint8_t)((b << 3) | (b >> 2));
    }
    fwrite(row.data(), 1, row.size(), f);
  }
  fclose(f);
}

int main(int argc, char **argv) {
  const char *outdir = argc > 1 ? argv[1] : "/tmp";

  const int sizes[] = {480, 320, 240, 160, 120};
  const int nsizes = sizeof(sizes) / sizeof(sizes[0]);

  printf("================================================================\n");
  printf(" SDF 骰子渲染器 · 离线验证（Mac）\n");
  printf("================================================================\n\n");

  // ---------------------------------------------------------------- 1) 正确性
  printf("【1】6 个点数落位后「朝上的面」是否正确\n");
  bool allOk = true;
  for (int v = 1; v <= 6; v++) {
    Quat q = targetForValue(v, 0.0f);
    // 哪个面法线的 +Y 分量最大 = 朝上的面
    int bestVal = 0; float bestY = -2.0f;
    for (int i = 0; i < 6; i++) {
      V3 n = qrot(FACES[i].n, q);
      if (n.y > bestY) { bestY = n.y; bestVal = FACES[i].val; }
    }
    bool ok = (bestVal == v);
    if (!ok) allOk = false;
    printf("   点数 %d → 顶面 %d   upness=%.3f   %s\n",
           v, bestVal, bestY, ok ? "OK" : "**不符**");
  }
  printf("   %s\n\n", allOk ? "全部正确 ✓" : "有错误 ✗");

  // ---------------------------------------------------------------- 2) 各分辨率耗时
  printf("【2】各分辨率的渲染耗时（Mac，仅作相对参考）\n");
  printf("   %-8s %-10s %-10s %-12s %-12s\n",
         "分辨率", "ms/帧", "FPS", "步进像素", "SDF求值");

  std::vector<uint16_t> big;
  int Wbig = 0, Hbig = 0;

  for (int si = 0; si < nsizes; si++) {
    int W = sizes[si], H = sizes[si];
    std::vector<uint16_t> buf((size_t)W * H);

    Renderer r;
    r.W = W; r.H = H; r.buf = buf.data();

    Quat q = targetForValue(3, 0.0f);

    // 预热
    r.render(q, 0.0f);

    const int N = 10;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; i++) r.render(q, 0.0f);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / N;

    printf("   %-8s %-10.2f %-10.1f %-12u %-12u\n",
           (std::to_string(W) + "²").c_str(), ms, 1000.0 / ms,
           r.statPixels, r.statSdf);

    if (W == 240) { big = buf; Wbig = W; Hbig = H; }
  }
  printf("\n");

  // ---------------------------------------------------------------- 3) 输出图片
  printf("【3】输出各点数落位的渲染图（240×240）\n");
  {
    std::vector<uint16_t> buf((size_t)240 * 240);
    Renderer r;
    r.W = 240; r.H = 240; r.buf = buf.data();
    for (int v = 1; v <= 6; v++) {
      Quat q = targetForValue(v, 0.0f);
      r.render(q, 0.0f);
      char path[512];
      snprintf(path, sizeof(path), "%s/sdf_240_val%d.ppm", outdir, v);
      writePPM(path, buf.data(), 240, 240);
      printf("   %s\n", path);
    }
  }

  // ---------------------------------------------------------------- 4) 关键统计
  printf("\n【4】每像素成本分析（240² 满屏，斜视姿态）\n");
  {
    std::vector<uint16_t> buf((size_t)240 * 240);
    Renderer r;
    r.W = 240; r.H = 240; r.buf = buf.data();
    Quat q = targetForValue(6, 0.0f);
    r.render(q, 0.0f);

    double perPixSteps = r.statPixels ? (double)r.statSteps / r.statPixels : 0;
    double perPixSdf   = r.statPixels ? (double)r.statSdf   / r.statPixels : 0;
    printf("   包围盒内像素     %u\n", r.statPixels);
    printf("   命中像素         %u\n", r.statHits);
    printf("   总步进次数       %u\n", r.statSteps);
    printf("   平均步进/像素    %.2f  (上限 maxSteps=56)\n", perPixSteps);
    printf("   SDF 求值/像素    %.2f  (步进 + 4 次法线 + 4 次AO)\n", perPixSdf);
    printf("\n   → 若 ESP32-S3 单次 SDF 求值约 X 微秒，\n");
    printf("     则 240² 一帧 ≈ %.0f 次求值 × X\n", perPixSdf * r.statHits);
  }

  printf("\n================================================================\n");
  return allOk ? 0 : 1;
}
