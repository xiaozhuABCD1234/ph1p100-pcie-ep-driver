/* frame_decode.c — FPGA↔RK 非压缩帧合同的 **C 参考实现**（RK 侧可直接照抄）
 *
 * 覆盖：帧头解析 + CRC32(IEEE 反射) + RAW10 解包 + 3x3 双线性去马赛克(保色夹取) + Y8。
 * 语义与 `parallel/algorithms/image_algorithms_ref.py` **逐像素一致**（由 run_c_decoder.py 断言）。
 *
 * 用法：frame_decode <frame.bin> <out.y8> [out.pgm]
 *   退出码：0=OK，2=格式/CRC 失败，3=用法/IO 失败
 *
 * 许可：本项目自用。可被 RK 侧改写为内核/用户态代码。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define HDR_BYTES 128
#define CRC_OFF   0x58

static uint32_t crc_table[256];
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc_table[i] = c;
    }
}
static uint32_t crc32_ieee(const uint8_t *p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) c = crc_table[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

static uint32_t rd_u32(const uint8_t *b, size_t off) {
    return (uint32_t)b[off] | ((uint32_t)b[off+1] << 8) |
           ((uint32_t)b[off+2] << 16) | ((uint32_t)b[off+3] << 24);
}
static uint16_t rd_u16(const uint8_t *b, size_t off) {
    return (uint16_t)((uint16_t)b[off] | ((uint16_t)b[off+1] << 8));
}

/* 与 Python 的 x % 2 对齐（负数也返回 0/1） */
static int parity(int v) { int m = v % 2; return m < 0 ? m + 2 : m; }

static const int8_t *CFA4(int phase) {
    /* 每个相位 4 个位置(TL,TR,BL,BR)的颜色: 0=R 1=G 2=B */
    static const int8_t t[4][4] = {
        {0,1,1,2},   /* RGGB */
        {2,1,1,0},   /* BGGR */
        {1,0,2,1},   /* GRBG */
        {1,2,0,1},   /* GBRG */
    };
    return t[phase & 3];
}
static int cfa_color(int y, int x, int phase) {
    const int8_t *c = CFA4(phase);
    int row = parity(y), col = parity(x);
    if (row == 0) return col == 0 ? c[0] : c[1];
    return col == 0 ? c[2] : c[3];
}

/* 保色夹取：夹回图像并保持 CFA 奇偶；某维只有 1 像素时退回中心 */
static void nb(int x, int y, int dx, int dy, int w, int h,
               const uint16_t *raw, int *ox, int *oy) {
    int tx = x + dx, ty = y + dy;
    int cx = tx < 0 ? 0 : (tx > w - 1 ? w - 1 : tx);
    if (parity(cx) != parity(tx)) {
        if (cx + 1 <= w - 1) cx++;
        else if (cx - 1 >= 0) cx--;
        else { *ox = x; *oy = y; return; }
    }
    int cy = ty < 0 ? 0 : (ty > h - 1 ? h - 1 : ty);
    if (parity(cy) != parity(ty)) {
        if (cy + 1 <= h - 1) cy++;
        else if (cy - 1 >= 0) cy--;
        else { *ox = x; *oy = y; return; }
    }
    *ox = cx; *oy = cy;
}

static int avg2(int a, int b) { return (a + b + 1) >> 1; }
static int avg4(int a, int b, int c, int d) { return (a + b + c + d + 2) >> 2; }

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <frame.bin> <out.y8> [out.pgm]\n", argv[0]); return 3; }
    crc_init();

    FILE *f = fopen(argv[1], "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", argv[1]); return 3; }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 3; }
    long sz = ftell(f);
    if (sz < HDR_BYTES) { fclose(f); fprintf(stderr, "too short\n"); return 2; }
    uint8_t *buf = (uint8_t *)malloc((size_t)sz);
    fseek(f, 0, SEEK_SET);
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) { fclose(f); free(buf); return 3; }
    fclose(f);

    if (memcmp(buf, "R10F", 4) != 0) { fprintf(stderr, "bad magic\n"); free(buf); return 2; }
    if (rd_u16(buf, 0x08) != HDR_BYTES) { fprintf(stderr, "header_bytes != 128\n"); free(buf); return 2; }
    uint32_t stored_hcrc = rd_u32(buf, CRC_OFF);
    uint8_t saved[4]; memcpy(saved, buf + CRC_OFF, 4); memset(buf + CRC_OFF, 0, 4);
    uint32_t hcrc = crc32_ieee(buf, HDR_BYTES);
    memcpy(buf + CRC_OFF, saved, 4);
    if (hcrc != stored_hcrc) { fprintf(stderr, "header crc mismatch\n"); free(buf); return 2; }

    int fmt = buf[0x0B];
    int phase = buf[0x28];
    int w = rd_u16(buf, 0x1C), h = rd_u16(buf, 0x1E);
    uint32_t stride = rd_u32(buf, 0x20), raw_bytes = rd_u32(buf, 0x2C), payload_bytes = rd_u32(buf, 0x30);
    if (fmt != 0) { fprintf(stderr, "this reference handles RAW10 packed only (fmt=%d)\n", fmt); free(buf); return 2; }
    if (w <= 0 || h <= 0 || (w % 4) != 0 || stride != (uint32_t)(w / 4 * 5) ||
        raw_bytes != stride * (uint32_t)h || payload_bytes < raw_bytes) {
        fprintf(stderr, "geometry inconsistent\n"); free(buf); return 2;
    }
    if ((long)(HDR_BYTES + payload_bytes) > sz) { fprintf(stderr, "payload truncated\n"); free(buf); return 2; }
    const uint8_t *pl = buf + HDR_BYTES;
    if (crc32_ieee(pl, raw_bytes) != rd_u32(buf, 0x50) ||
        crc32_ieee(pl, payload_bytes) != rd_u32(buf, 0x54)) {
        fprintf(stderr, "payload crc mismatch\n"); free(buf); return 2;
    }

    size_t n = (size_t)w * (size_t)h;
    uint16_t *raw = (uint16_t *)malloc(n * sizeof(uint16_t));
    for (size_t i = 0, o = 0, k = 0; i < n; i += 4, o += 5) {
        uint8_t b0 = pl[o], b1 = pl[o+1], b2 = pl[o+2], b3 = pl[o+3], b4 = pl[o+4];
        raw[k++] = (uint16_t)((b0 << 2) | (b4 & 3));
        raw[k++] = (uint16_t)((b1 << 2) | ((b4 >> 2) & 3));
        raw[k++] = (uint16_t)((b2 << 2) | ((b4 >> 4) & 3));
        raw[k++] = (uint16_t)((b3 << 2) | ((b4 >> 6) & 3));
    }

    uint8_t *y8 = (uint8_t *)malloc(n);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int own = cfa_color(y, x, phase), r, g, b;
            int x1, y1, x2, y2, x3, y3, x4, y4;
            if (own == 1) {
                int horiz = cfa_color(y, x - 1, phase);   /* 未夹取，x=0 也正确 */
                if (horiz == 0) {
                    nb(x, y, -1, 0, w, h, raw, &x1, &y1); nb(x, y, 1, 0, w, h, raw, &x2, &y2);
                    nb(x, y, 0, -1, w, h, raw, &x3, &y3); nb(x, y, 0, 1, w, h, raw, &x4, &y4);
                    r = avg2(raw[y1*w+x1], raw[y2*w+x2]);
                    b = avg2(raw[y3*w+x3], raw[y4*w+x4]);
                } else {
                    nb(x, y, -1, 0, w, h, raw, &x1, &y1); nb(x, y, 1, 0, w, h, raw, &x2, &y2);
                    nb(x, y, 0, -1, w, h, raw, &x3, &y3); nb(x, y, 0, 1, w, h, raw, &x4, &y4);
                    b = avg2(raw[y1*w+x1], raw[y2*w+x2]);
                    r = avg2(raw[y3*w+x3], raw[y4*w+x4]);
                }
                g = raw[y*w+x];
            } else {
                nb(x, y, 0, -1, w, h, raw, &x1, &y1); nb(x, y, 0, 1, w, h, raw, &x2, &y2);
                nb(x, y, -1, 0, w, h, raw, &x3, &y3); nb(x, y, 1, 0, w, h, raw, &x4, &y4);
                g = avg4(raw[y1*w+x1], raw[y2*w+x2], raw[y3*w+x3], raw[y4*w+x4]);
                nb(x, y, -1, -1, w, h, raw, &x1, &y1); nb(x, y, 1, -1, w, h, raw, &x2, &y2);
                nb(x, y, -1, 1, w, h, raw, &x3, &y3); nb(x, y, 1, 1, w, h, raw, &x4, &y4);
                int diag = avg4(raw[y1*w+x1], raw[y2*w+x2], raw[y3*w+x3], raw[y4*w+x4]);
                if (own == 0) { r = raw[y*w+x]; b = diag; }
                else          { b = raw[y*w+x]; r = diag; }
            }
            int r8 = r >> 2, g8 = g >> 2, b8 = b >> 2;
            y8[(size_t)y*w + x] = (uint8_t)((77*r8 + 150*g8 + 29*b8 + 128) >> 8);
        }
    }

    FILE *o = fopen(argv[2], "wb");
    if (!o) { fprintf(stderr, "cannot write %s\n", argv[2]); free(buf); free(raw); free(y8); return 3; }
    fwrite(y8, 1, n, o); fclose(o);
    if (argc >= 4) {
        FILE *p = fopen(argv[3], "wb");
        if (p) { fprintf(p, "P5\n%d %d\n255\n", w, h); fwrite(y8, 1, n, p); fclose(p); }
    }
    unsigned long long ysum = 0;
    for (size_t i = 0; i < n; i++) ysum += y8[i];
    printf("C_DECODE_OK w=%d h=%d phase=%d y8_sum=%llu\n", w, h, phase, ysum);
    free(buf); free(raw); free(y8);
    return 0;
}
