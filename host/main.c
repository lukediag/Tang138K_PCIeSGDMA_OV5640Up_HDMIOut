// ============================================================================
// main.c — sgdmaVideoShakeHand v1.5 视频采集 (环形 8 帧 + BAR2 握手)
// ============================================================================
// v1.4 双线程架构:
//   - 握手线程: 10μs busy-wait, 每帧重配 SGDMA (完成检测 + 启动新帧)
//   - 渲染线程: 取滞后 3 帧的稳定 buffer → SDL 渲染
//   - ★ -headless 模式: 跳过 SDL2, 纯采集 + 每 10 帧命令行打印 FPS
// v1.5 改动:
//   - 环形 8 帧 (滞后 3 帧, 保护窗口 4 帧/65.6ms)
//   - get_display_frame 跳过 16B magic header
//   - 注: 曾尝试 SCHED_FIFO/nice 实时优先级, 因昇腾 cgroup 拒绝且 nice 饿死
//     渲染线程, 已回退为普通优先级 (偶尔撕裂待后续用 FIFO 深度/丢帧重对齐解决)
//
// 编译: make
// 运行:
//   sudo ./bin/video_gui -headless   # 纯采集 (无显示, 验证采集链路性能)
//   sudo ./bin/video_gui -v          # GUI + DEBUG 日志
// ============================================================================

#define _GNU_SOURCE   // pthread_setaffinity_np 需要 (必须在所有 #include 之前)

#include "utils/sgdma_core.h"
#include "utils/log.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <signal.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

// ---- v1.4 双线程: 握手线程全局上下文 ----
static SGDMAContext *g_hs_ctx;      // 握手线程访问的 ctx
static volatile int  g_hs_stop;     // 1=停止握手线程

// 握手线程入口: usleep 温和轮询 (不烧 CPU, 避免饿死 SDL 渲染线程)
// ★ v1.5.1: 从 10μs busy-wait 改回 usleep — busy-wait 占 100% CPU,
//   和软件渲染抢核导致 GUI 掉到 20fps + 撕裂。usleep 省 CPU, 帧率恢复。
static void *handshake_thread_fn(void *arg) {
    (void)arg;

    // ★ v1.5.3: 绑核 0 — 握手线程独占一个核, 避免被 SDL 渲染线程抢占 CPU
    //   导致 usleep 调度延迟 >16.7ms → 漏帧 → SGDMA 从 FIFO 中间搬 → 画面错位
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
        SLOG_WARN("handshake thread: bind CPU0 failed (continuing unbound)");

    while (!g_exit && !g_hs_stop) {
        sgdma_handshake_tick(g_hs_ctx);
        usleep(HANDSHAKE_POLL_US);   // 100μs 温和轮询
    }
    return NULL;
}

// ---- FPS 追踪 (滑动窗口平均) ----
#define FPS_WINDOW 30

static struct {
    double frame_times[FPS_WINDOW];
    int    idx;
    int    count;
    double fps;
    struct timespec last_ts;
} fps_ctx;

static void fps_reset(void) {
    memset(&fps_ctx, 0, sizeof(fps_ctx));
    clock_gettime(CLOCK_MONOTONIC, &fps_ctx.last_ts);
}

static void fps_tick(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = (now.tv_sec  - fps_ctx.last_ts.tv_sec)
              + (now.tv_nsec - fps_ctx.last_ts.tv_nsec) * 1e-9;
    fps_ctx.last_ts = now;

    fps_ctx.frame_times[fps_ctx.idx % FPS_WINDOW] = dt;
    fps_ctx.idx++;
    if (fps_ctx.count < FPS_WINDOW) fps_ctx.count++;

    if (fps_ctx.count > 0) {
        double sum = 0.0;
        int n = fps_ctx.count < FPS_WINDOW ? fps_ctx.count : FPS_WINDOW;
        for (int i = 0; i < n; i++) sum += fps_ctx.frame_times[i];
        fps_ctx.fps = (sum > 0.0) ? (double)n / sum : 0.0;
    }
}

// ---- RGB565 → BGRA 转换 (查表法加速) ----
// ★ v1.1: FPGA 直传 RGB565 (2B/像素), host 转 BGRA 供 SDL 上屏
// ★ v2.0 优化: 65536 项 BGRA 查找表, 每像素 1 次查表 + 1 次 32bit 写
//   (原逐像素 ~20 次位运算 → 2 次/像素, 提速 5~10×)
//   内存字节序 (小端): 低字节 = G[2:0]B[4:0], 高字节 = R[4:0]G[5:3]
static uint32_t rgb565_bgra_lut[65536];
static int      rgb565_lut_ready = 0;

static void rgb565_lut_init(void) {
    if (rgb565_lut_ready) return;
    for (int i = 0; i < 65536; i++) {
        int r5 = (i >> 11) & 0x1F;
        int g6 = (i >> 5)  & 0x3F;
        int b5 = i & 0x1F;
        // 5/6/5 → 8/8/8 位扩展 (左移 + 高位复制)
        int r = (r5 << 3) | (r5 >> 2);
        int g = (g6 << 2) | (g6 >> 4);
        int b = (b5 << 3) | (b5 >> 2);
        // BGRA (小端内存 B,G,R,A = uint32 A<<24|R<<16|G<<8|B)
        rgb565_bgra_lut[i] = 0xFF000000u | ((uint32_t)r << 16)
                           | ((uint32_t)g << 8)  | (uint32_t)b;
    }
    rgb565_lut_ready = 1;
}

static void rgb565_to_bgra(const uint8_t *src, uint8_t *dst, int width, int height) {
    rgb565_lut_init();
    int total = width * height;
    uint32_t *dst32 = (uint32_t *)dst;
    for (int i = 0; i < total; i++) {
        uint16_t px = (uint16_t)src[i*2] | ((uint16_t)src[i*2+1] << 8);  // 小端
        dst32[i] = rgb565_bgra_lut[px];
    }
}

// ---- RGB565 时域降噪 (2 帧平均 + 运动阈值) ----
// ★ v1.1.1: 静止区 2 帧平均降噪, 运动区保持当前帧 (避免鬼影)
//   差异 = |ΔR| + |ΔG| + |ΔB| (RGB565 域, 范围 0~125), < 阈值 → 平均
static void denoise_rgb565(const uint8_t *cur, const uint8_t *prev,
                           uint8_t *out, int width, int height, int threshold) {
    int total = width * height;
    for (int i = 0; i < total; i++) {
        uint16_t c = (uint16_t)(cur[i*2] | (cur[i*2+1] << 8));
        uint16_t p = (uint16_t)(prev[i*2] | (prev[i*2+1] << 8));

        int cr = (c >> 11) & 0x1F, cg = (c >> 5) & 0x3F, cb = c & 0x1F;
        int pr = (p >> 11) & 0x1F, pg = (p >> 5) & 0x3F, pb = p & 0x1F;

        int diff = abs(cr - pr) + abs(cg - pg) + abs(cb - pb);
        uint16_t r;
        if (diff < threshold) {
            // 静止区: 逐通道平均降噪
            int r5 = (cr + pr) >> 1;
            int g6 = (cg + pg) >> 1;
            int b5 = (cb + pb) >> 1;
            r = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
        } else {
            // 运动区: 保持当前帧, 不拖影
            r = c;
        }
        out[i*2]   = (uint8_t)(r & 0xFF);
        out[i*2+1] = (uint8_t)(r >> 8);
    }
}

// ---- 信号处理 ----
static void on_signal(int sig) { (void)sig; g_exit = 1; }

// ---- 字体查找 ----
static const char *FONT_CANDIDATES[] = {
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
    "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
    NULL
};

static TTF_Font *load_font(int size) {
    for (int i = 0; FONT_CANDIDATES[i]; i++) {
        TTF_Font *f = TTF_OpenFont(FONT_CANDIDATES[i], size);
        if (f) {
            SLOG_INFO("Font loaded: %s (size=%d)", FONT_CANDIDATES[i], size);
            return f;
        }
    }
    SLOG_WARN("No monospace font found, FPS text may be unavailable");
    return NULL;
}

// ============================================================================
// 主程序
// ============================================================================
int main(int argc, char *argv[]) {
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    // ---- 解析参数 ----
    const char *devnode  = DEFAULT_DEVNODE;
    SgdmaLogLevel log_lvl = SGDMA_LOG_INFO;
    int headless = 0;                       // ★ -headless: 跳过 SDL2, 纯采集
    int sx = 0;                             // ★ -sx: 时域降噪开关 (2帧平均+运动阈值)
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-d") && i + 1 < argc) {
            devnode = argv[++i];
        } else if (!strcmp(argv[i], "-vvb")) {
            log_lvl = SGDMA_LOG_TRACE;
            g_save_bmp = 1;
            g_frame_dump = 1;      // ★ v2.0: 每帧 dump 帧头帧尾分析
        } else if (!strcmp(argv[i], "-vv")) {
            log_lvl = SGDMA_LOG_TRACE;
            g_frame_dump = 1;      // ★ v2.0: 每帧 dump 帧头帧尾分析
        } else if (!strcmp(argv[i], "-v")) {
            log_lvl = SGDMA_LOG_DEBUG;
        } else if (!strcmp(argv[i], "-sx")) {
            sx = 1;                         // ★ 时域降噪开关
        } else if (!strcmp(argv[i], "-headless")) {
            headless = 1;                    // ★ 纯采集模式
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr, "Usage: %s [-d <devnode>] [-v] [-vv] [-vvb] [-headless] [-sx]\n", argv[0]);
            fprintf(stderr, "  -d    device node (default: %s)\n", DEFAULT_DEVNODE);
            fprintf(stderr, "  -v    verbose (DEBUG level logging)\n");
            fprintf(stderr, "  -vv   very verbose (TRACE level logging)\n");
            fprintf(stderr, "  -vvb  TRACE + save each frame as BMP to ./frames/\n");
            fprintf(stderr, "  -headless  no SDL2 display, capture only (print FPS every 10 frames)\n");
            fprintf(stderr, "  -sx   enable temporal denoise (2-frame avg + motion threshold)\n");
            fprintf(stderr, "  ESC or close window to exit\n");
            return 0;
        }
    }

    // ========================================================================
    // 0. 初始化日志系统
    // ========================================================================
    sgdma_log_init("./logs", "sgdma_gui", log_lvl);
    SLOG_INFO("==============================================");
    SLOG_INFO("  SGDMA_OV5640 v2.0 — OV5640 摄像头采集 (720p RGB565 直传)");
    SLOG_INFO("  视频: %d×%d | 帧: %d 字节 (%.1f MB)",
              VIDEO_WIDTH, VIDEO_HEIGHT, FRAME_SIZE, FRAME_SIZE / 1048576.0);
    SLOG_INFO("  模式: %s%s", headless ? "HEADLESS (纯采集)" : "GUI (SDL2 显示)",
              sx ? " + 时域降噪" : "");
    SLOG_INFO("  日志级别: %s", log_lvl == SGDMA_LOG_DEBUG ? "DEBUG" :
                                  log_lvl == SGDMA_LOG_TRACE ? "TRACE" : "INFO");
    SLOG_INFO("  日志文件: %s", sgdma_log_file_path() ? sgdma_log_file_path() : "(仅终端)");
    if (g_save_bmp) SLOG_INFO("  BMP 保存: 已启用 (全部帧 → ./frames/)");

    // ========================================================================
    // 1. 初始化 SGDMA
    // ========================================================================
    SGDMAContext ctx;
    if (sgdma_init(&ctx, devnode) != 0) {
        SLOG_ERROR("SGDMA initialization failed, exiting");
        sgdma_log_close();
        return 1;
    }

    // ========================================================================
    // 2. ★ Benchmark: SGDMA C2H 读测速 + BAR2 R/W 延迟
    // ========================================================================
    SLOG_INFO("");
    SLOG_INFO("=== 启动前基准测试 ===");
    sgdma_benchmark_bar2_latency(&ctx);
    SLOG_INFO("=== 基准测试完成 ===");
    SLOG_INFO("");

    // ========================================================================
    // 3. 初始化 SDL2 (headless 模式跳过)
    // ========================================================================
    SDL_Window   *window   = NULL;
    SDL_Renderer *renderer = NULL;
    SDL_Texture  *video_tex = NULL;
    TTF_Font     *font     = NULL;

    // ★ FPS 文字纹理缓存 (避免每帧重建 TTF 纹理 = 巨大 CPU 开销)
    SDL_Texture  *text_tex[1] = {NULL};
    SDL_Rect      text_rect[1] = {0};
    uint32_t      text_updated_frame = 0;
    uint8_t      *rgb_buf   = NULL;   // ★ v1.1: RGB565→BGRA 转换缓冲
    uint8_t      *prev_frame = NULL;  // ★ -sx: 上一帧 RGB565 (降噪参考)
    uint8_t      *denoised_buf = NULL; // ★ -sx: 降噪输出 RGB565

    // ★ v1.1: RGB565→BGRA 转换缓冲 (headless BMP 与 GUI 渲染共用)
    rgb_buf = malloc((size_t)VIDEO_WIDTH * VIDEO_HEIGHT * 4);
    if (!rgb_buf)
        SLOG_WARN("无法分配 RGB565→BGRA 转换缓冲 (BMP/GUI 降级)");

    // ★ -sx: 时域降噪缓冲 (上一帧参考 + 降噪输出, 各 1 帧 RGB565)
    if (sx) {
        prev_frame = calloc(1, FRAME_SIZE);       // 清零: 首帧判为全运动 → 用当前帧
        denoised_buf = malloc(FRAME_SIZE);
        if (!prev_frame || !denoised_buf) {
            SLOG_WARN("无法分配时域降噪缓冲, 降噪关闭");
            sx = 0;
            free(prev_frame); prev_frame = NULL;
            free(denoised_buf); denoised_buf = NULL;
        }
    }

    if (!headless) {
        // ★ v1.4.2: 打印 SDL 环境 (版本 + 编译进 SDL 的 video drivers)
        SDL_version sdl_ver;
        SDL_GetVersion(&sdl_ver);
        SLOG_INFO("SDL version: %d.%d.%d", sdl_ver.major, sdl_ver.minor, sdl_ver.patch);
        int ndrv = SDL_GetNumVideoDrivers();
        SLOG_INFO("SDL video drivers (%d):", ndrv);
        for (int i = 0; i < ndrv; i++)
            SLOG_INFO("  - %s", SDL_GetVideoDriver(i));

        SLOG_INFO("Initializing SDL2...");
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            SLOG_ERROR("SDL_Init: %s", SDL_GetError());
            sgdma_cleanup(&ctx);
            sgdma_log_close();
            return 1;
        }
        SLOG_INFO("SDL video driver in use: %s", SDL_GetCurrentVideoDriver());

        if (TTF_Init() != 0) {
            SLOG_ERROR("TTF_Init: %s", TTF_GetError());
            SDL_Quit();
            sgdma_cleanup(&ctx);
            sgdma_log_close();
            return 1;
        }

        window = SDL_CreateWindow(
            "sgdmaVideoShakeHand v1.4 - BAR2 Handshake",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            VIDEO_WIDTH, VIDEO_HEIGHT,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!window) {
            SLOG_ERROR("SDL_CreateWindow: %s", SDL_GetError());
            TTF_Quit(); SDL_Quit();
            sgdma_cleanup(&ctx);
            sgdma_log_close();
            return 1;
        }

        // ★ v1.5.2: 视频显示是纯 2D blit, 强制软件渲染 (SDL_RENDERER_SOFTWARE),
        //   绕开 ACCELERATED → GLX → llvmpipe 软渲染 3D 管线 (最慢路径)
        // ★ v2.0: 加 vsync — 让 present 对齐显示器 vblank, 消除上屏撕裂
        SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
        renderer = SDL_CreateRenderer(window, -1,
            SDL_RENDERER_SOFTWARE | SDL_RENDERER_PRESENTVSYNC);
        if (!renderer) {
            SLOG_WARN("Software renderer failed (%s), trying accelerated...", SDL_GetError());
            renderer = SDL_CreateRenderer(window, -1,
                SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        }
        if (renderer) {
            SDL_RendererInfo ri;
            if (SDL_GetRendererInfo(renderer, &ri) == 0)
                SLOG_INFO("SDL renderer: %s", ri.name);
        }
        if (!renderer) {
            SLOG_ERROR("SDL_CreateRenderer: %s", SDL_GetError());
            SDL_DestroyWindow(window);
            TTF_Quit(); SDL_Quit();
            sgdma_cleanup(&ctx);
            sgdma_log_close();
            return 1;
        }

        // ★ 零拷贝 BGRA 纹理: SDL_UpdateTexture 直接消费帧缓冲, 无格式转换
        video_tex = SDL_CreateTexture(renderer,
            SDL_PIXELFORMAT_BGRA32,
            SDL_TEXTUREACCESS_STREAMING,
            VIDEO_WIDTH, VIDEO_HEIGHT);
        if (!video_tex) {
            SLOG_WARN("BGRA32 streaming failed, trying ARGB8888...");
            video_tex = SDL_CreateTexture(renderer,
                SDL_PIXELFORMAT_ARGB8888,
                SDL_TEXTUREACCESS_STREAMING,
                VIDEO_WIDTH, VIDEO_HEIGHT);
        }
        if (!video_tex) {
            SLOG_ERROR("Cannot create video texture: %s", SDL_GetError());
            SDL_DestroyRenderer(renderer); SDL_DestroyWindow(window);
            TTF_Quit(); SDL_Quit();
            sgdma_cleanup(&ctx);
            sgdma_log_close();
            return 1;
        }
        SDL_SetTextureBlendMode(video_tex, SDL_BLENDMODE_NONE);

        // 字体
        font = load_font(28);
    }

    // ========================================================================
    // 4. ★ 双线程: 握手线程 + 渲染主循环 (v1.4 环形 5 帧)
    // ========================================================================
    // 握手线程: 10μs busy-wait, 每帧重配 SGDMA (完成检测 + 启动新帧)
    // 渲染线程: 取滞后 3 帧的稳定 buffer → SDL 渲染 (headless 模式只统计)
    // ========================================================================
    SLOG_INFO("Starting dual-thread handshake (%s mode)",
              headless ? "headless" : "GUI");

    // ★ v1.1.x: stream_reset 已在 sgdma_init 内完成 (清 FIFO + header_inserter),
    //   gate 在 vsync↓ 帧边界开门, 第一帧从干净帧头开始

    // ---- 握手线程入口 ----
    g_hs_ctx = &ctx;
    g_hs_stop = 0;

    // ★ v1.5.3: 渲染线程(主线程)绑核 1 — 与握手线程(核0)分离, 消除 CPU 竞争
    {
        cpu_set_t render_set;
        CPU_ZERO(&render_set);
        CPU_SET(1, &render_set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(render_set), &render_set) != 0)
            SLOG_WARN("render thread: bind CPU1 failed (continuing unbound)");
    }

    pthread_t hs_thread;
    int th_ret = pthread_create(&hs_thread, NULL, handshake_thread_fn, NULL);
    if (th_ret != 0) {
        SLOG_ERROR("pthread_create failed: %d", th_ret);
    }

    fps_reset();
    uint32_t frame_ok   = 0;
    uint32_t frame_skip = 0;
    uint32_t frame_err  = 0;
    uint16_t last_fid   = 0;
    uint8_t *render_ptr = NULL;

    struct timespec loop_start;
    clock_gettime(CLOCK_MONOTONIC, &loop_start);

    while (!g_exit) {
        if (headless) {
            // ============================================================
            // ★ headless 模式: 纯采集, 无 SDL2, 每 10 帧打印 FPS
            // ============================================================
            uint16_t frame_id;
            uint8_t *frame = sgdma_get_display_frame(&ctx, &frame_id);
            if (frame && frame != render_ptr) {
                render_ptr = frame;
                frame_ok++;

                // BMP 保存 (-vvb, RGB565 → BGRA 转换后保存)
                if (g_save_bmp && rgb_buf) {
                    rgb565_to_bgra(frame, rgb_buf, VIDEO_WIDTH, VIDEO_HEIGHT);
                    sgdma_save_frame_bmp(rgb_buf, frame_id,
                                         VIDEO_WIDTH, VIDEO_HEIGHT);
                }

                // 帧跳检测
                if (frame_ok > 1 &&
                    frame_id != last_fid && last_fid > 0 &&
                    (uint16_t)(frame_id - last_fid) > 1) {
                    frame_skip += (uint16_t)(frame_id - last_fid - 1);
                }
                last_fid = frame_id;

                fps_tick();

                // ★ 每 10 帧打印一次 FPS
                if (frame_ok % 10 == 0) {
                    SLOG_INFO("FPS %.1f | frames=%u | skip=%u | err=%u | id=%u",
                              fps_ctx.fps, frame_ok, frame_skip, frame_err, last_fid);
                }
            }
            // 让出 CPU (握手线程已 10μs busy-wait, 主线程 1ms 足够)
            usleep(1000);
            continue;
        }

        // ============================================================
        // GUI 模式
        // ============================================================
        // ---- 事件处理 ----
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) g_exit = 1;
            if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_ESCAPE ||
                    ev.key.keysym.sym == SDLK_q) g_exit = 1;
            }
        }

        // ---- ★ v1.4: 取滞后 3 帧的显示帧 (非阻塞, 稳定不被覆盖) ----
        uint16_t frame_id;
        uint8_t *frame = sgdma_get_display_frame(&ctx, &frame_id);
        if (frame && frame != render_ptr) {
            render_ptr = frame;
            frame_ok++;

            // BMP 保存 (-vvb, RGB565 → BGRA 转换后保存)
            if (g_save_bmp && rgb_buf) {
                rgb565_to_bgra(frame, rgb_buf, VIDEO_WIDTH, VIDEO_HEIGHT);
                sgdma_save_frame_bmp(rgb_buf, frame_id,
                                     VIDEO_WIDTH, VIDEO_HEIGHT);
            }

            // 帧跳检测
            if (frame_ok > 1 &&
                frame_id != last_fid && last_fid > 0 &&
                (uint16_t)(frame_id - last_fid) > 1) {
                frame_skip += (uint16_t)(frame_id - last_fid - 1);
            }
            last_fid = frame_id;

            fps_tick();

            // ---- ★ v2.0 优化: 只有新帧才转换 + UpdateTexture (消除每轮重复转换) ----
            if (sx && prev_frame && denoised_buf) {
                // ★ -sx: 时域降噪 → 转换 → 上屏
                denoise_rgb565(render_ptr, prev_frame, denoised_buf,
                               VIDEO_WIDTH, VIDEO_HEIGHT, 16);
                memcpy(prev_frame, render_ptr, FRAME_SIZE);  // 保存当前帧供下帧参考
                rgb565_to_bgra(denoised_buf, rgb_buf, VIDEO_WIDTH, VIDEO_HEIGHT);
            } else {
                rgb565_to_bgra(render_ptr, rgb_buf, VIDEO_WIDTH, VIDEO_HEIGHT);
            }
            SDL_UpdateTexture(video_tex, NULL, rgb_buf,
                              VIDEO_WIDTH * 4);
        }
        // 无新帧: 直接复用上一帧纹理 (只 RenderCopy, 不重复转换)

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, video_tex, NULL, NULL);

        // ---- FPS 叠加 (每 15 帧重建一次, 帧间复用缓存纹理) ----
        if (font) {
            const uint32_t TEXT_REFRESH_EVERY = 15;
            if (text_tex[0] == NULL ||
                frame_ok - text_updated_frame >= TEXT_REFRESH_EVERY) {

                char text[96];
                SDL_Color fg = {0, 255, 0, 255};
                SDL_Color bg = {0, 0, 0, 192};

                if (text_tex[0]) SDL_DestroyTexture(text_tex[0]);

                snprintf(text, sizeof(text), " FPS: %5.1f | frames %u | skip %u | err %u",
                         fps_ctx.fps, frame_ok, frame_skip, frame_err);

                SDL_Surface *surf = TTF_RenderText_Shaded(font, text, fg, bg);
                if (surf) {
                    text_tex[0] = SDL_CreateTextureFromSurface(renderer, surf);
                    text_rect[0] = (SDL_Rect){8, 8, surf->w, surf->h};
                    SDL_FreeSurface(surf);
                }
                text_updated_frame = frame_ok;
            }

            if (text_tex[0])
                SDL_RenderCopy(renderer, text_tex[0], NULL, &text_rect[0]);
        }

        SDL_RenderPresent(renderer);
    }

    // ========================================================================
    // 4. 统计 & 清理
    // ========================================================================
    struct timespec loop_end;
    clock_gettime(CLOCK_MONOTONIC, &loop_end);
    double elapsed = (loop_end.tv_sec - loop_start.tv_sec)
                   + (loop_end.tv_nsec - loop_start.tv_nsec) * 1e-9;

    SLOG_INFO("==============================================");
    SLOG_INFO("  运行统计 (%s):", headless ? "纯采集" : "GUI");
    SLOG_INFO("    采集帧数:   %u", frame_ok);
    SLOG_INFO("    跳帧数:     %u (frame_id 不连续)", frame_skip);
    SLOG_INFO("    错误数:     %u", frame_err);
    SLOG_INFO("    BAR2 读:    %.2f μs", ctx.bar2_read_us);
    SLOG_INFO("    BAR2 写:    %.2f μs", ctx.bar2_write_us);
    SLOG_INFO("    运行时长:   %.1f s", elapsed);
    SLOG_INFO("    平均 FPS:   %.1f", frame_ok > 0 ? frame_ok / elapsed : 0.0);
    SLOG_INFO("  正常退出.");

    // ---- 停止握手线程 ----
    g_hs_stop = 1;
    if (th_ret == 0) {
        pthread_join(hs_thread, NULL);
        SLOG_DEBUG("Handshake thread joined");
    }
    g_hs_ctx = NULL;

    if (!headless) {
        if (font) TTF_CloseFont(font);
        TTF_Quit();
        for (int i = 0; i < 1; i++)
            if (text_tex[i]) SDL_DestroyTexture(text_tex[i]);
        SDL_DestroyTexture(video_tex);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
    free(rgb_buf);
    free(prev_frame);
    free(denoised_buf);
    sgdma_cleanup(&ctx);
    sgdma_log_close();

    return 0;
}
