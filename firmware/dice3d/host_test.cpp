// host_test.cpp — run dice_sdf.h on macOS to check correctness and compare relative costs
//
// Build: g++ -O2 -std=c++17 -o /tmp/dice_host host_test.cpp
// Run: /tmp/dice_host [output-directory]
//
// Outputs:
//   sdf_<res>_<val>.ppm  rendered images for each resting face (ready to view)
//   console report of ms/frame and SDF evaluation count at each resolution
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
  if (!f) { printf("  Cannot open %s\n", path); return; }
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
  printf(" SDF die renderer · offline validation (macOS)\n");
  printf("================================================================\n\n");

  // ---------------------------------------------------------------- 1) Correctness
  printf("[1] Verify that the correct face points up for all six results\n");
  bool allOk = true;
  for (int v = 1; v <= 6; v++) {
    Quat q = targetForValue(v, 0.0f);
    // The face normal with the greatest +Y component is the upward-facing face.
    int bestVal = 0; float bestY = -2.0f;
    for (int i = 0; i < 6; i++) {
      V3 n = qrot(FACES[i].n, q);
      if (n.y > bestY) { bestY = n.y; bestVal = FACES[i].val; }
    }
    bool ok = (bestVal == v);
    if (!ok) allOk = false;
    printf("   Face value %d → top face %d   upness=%.3f   %s\n",
           v, bestVal, bestY, ok ? "OK" : "**MISMATCH**");
  }
  printf("   %s\n\n", allOk ? "All correct ✓" : "Errors found ✗");

  // ---------------------------------------------------------------- 2) Render time by resolution
  printf("[2] Render time by resolution (macOS, for relative comparison only)\n");
  printf("   %-8s %-10s %-10s %-12s %-12s\n",
         "Resolution", "ms/frame", "FPS", "Pixels marched", "SDF evaluations");

  std::vector<uint16_t> big;
  int Wbig = 0, Hbig = 0;

  for (int si = 0; si < nsizes; si++) {
    int W = sizes[si], H = sizes[si];
    std::vector<uint16_t> buf((size_t)W * H);

    Renderer r;
    r.W = W; r.H = H; r.buf = buf.data();

    Quat q = targetForValue(3, 0.0f);

    // Warm up
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

  // ---------------------------------------------------------------- 3) Write images
  printf("[3] Write rendered images for each resting face (240×240)\n");
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

  // ---------------------------------------------------------------- 4) Key statistics
  printf("\n[4] Per-pixel cost analysis (full 240² frame, angled pose)\n");
  {
    std::vector<uint16_t> buf((size_t)240 * 240);
    Renderer r;
    r.W = 240; r.H = 240; r.buf = buf.data();
    Quat q = targetForValue(6, 0.0f);
    r.render(q, 0.0f);

    double perPixSteps = r.statPixels ? (double)r.statSteps / r.statPixels : 0;
    double perPixSdf   = r.statPixels ? (double)r.statSdf   / r.statPixels : 0;
    printf("   Pixels in bounds     %u\n", r.statPixels);
    printf("   Hit pixels         %u\n", r.statHits);
    printf("   Total march steps       %u\n", r.statSteps);
    printf("   Average steps/pixel    %.2f  (maximum maxSteps=56)\n", perPixSteps);
    printf("   SDF evaluations/pixel    %.2f  (marching + 4 normal samples + 4 AO samples)\n", perPixSdf);
    printf("\n   → If one SDF evaluation takes about X microseconds on ESP32-S3,\n");
    printf("     then one 240² frame takes about %.0f evaluations × X\n", perPixSdf * r.statHits);
  }

  printf("\n================================================================\n");
  return allOk ? 0 : 1;
}
