// ============================================================================
// log.c — SGDMADDR3v0 日志系统实现
// ============================================================================

#include "log.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

// ============================================================================
// 内部状态
// ============================================================================
static struct {
    FILE        *fp;               // 日志文件句柄 (NULL = 仅控制台)
    SgdmaLogLevel level;          // 最低输出级别
    char         file_path[512];  // 日志文件完整路径
    int          color_enabled;   // 是否启用控制台颜色
    int          initialized;     // 是否已初始化
} g_log = {
    .fp            = NULL,
    .level         = SGDMA_LOG_INFO,
    .file_path     = {0},
    .color_enabled = 1,           // 默认开启
    .initialized   = 0,
};

// ---- ANSI 颜色码 ----
#define COLOR_RESET   "\033[0m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_RED     "\033[31m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_GRAY    "\033[90m"
#define COLOR_MAGENTA "\033[35m"

// 每个级别对应的标签和颜色
static const struct {
    const char *tag;       // 短标签: "ERR", "WRN", "INF", "DBG", "TRC"
    const char *color;     // ANSI 颜色码
} g_level_meta[] = {
    { "ERR", COLOR_RED     },  // SGDMA_LOG_ERROR
    { "WRN", COLOR_YELLOW  },  // SGDMA_LOG_WARN
    { "INF", COLOR_GREEN   },  // SGDMA_LOG_INFO
    { "DBG", COLOR_CYAN    },  // SGDMA_LOG_DEBUG
    { "TRC", COLOR_GRAY    },  // SGDMA_LOG_TRACE
};

// ============================================================================
// 内部辅助函数
// ============================================================================

// 获取带微秒精度的当前时间戳字符串: "2026-08-03 14:30:25.123456"
static void timestamp_str(char *buf, size_t bufsz) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm tm_buf;
    time_t sec = tv.tv_sec;
    localtime_r(&sec, &tm_buf);

    snprintf(buf, bufsz, "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
             tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
             (long)tv.tv_usec);
}

// 递归创建目录 (类似 mkdir -p)
static int mkdir_p(const char *path) {
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);
    size_t len = strlen(tmp);
    if (len > 0 && tmp[len - 1] == '/') tmp[len - 1] = '\0';

    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) return -1;
    return 0;
}

// 从文件路径中提取文件名部分 (用于控制台输出: 不显示全路径)
static const char *basename_s(const char *path) {
    const char *p = strrchr(path, '/');
    return p ? p + 1 : path;
}

// ============================================================================
// 公开 API
// ============================================================================

int sgdma_log_init(const char *log_dir, const char *prefix, SgdmaLogLevel level) {
    // 避免重复初始化
    if (g_log.initialized) {
        sgdma_log_close();
    }

    const char *dir  = (log_dir && *log_dir) ? log_dir : ".";
    const char *pref = (prefix && *prefix) ? prefix : "sgdma";

    // 递归创建日志目录
    if (mkdir_p(dir) != 0) {
        fprintf(stderr, "[LOG] WARNING: Cannot create log dir '%s': %s\n",
                dir, strerror(errno));
        // 不返回失败, 回退到 stderr-only
    }

    // 生成日志文件名: <prefix>_YYYYMMDD_HHMMSS.log
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_buf;
    time_t sec = tv.tv_sec;
    localtime_r(&sec, &tm_buf);

    char fname[128];
    snprintf(fname, sizeof(fname),
             "%s_%04d%02d%02d_%02d%02d%02d.log",
             pref,
             tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

    snprintf(g_log.file_path, sizeof(g_log.file_path), "%s/%s", dir, fname);

    // 打开日志文件 (追加模式, 万一同秒启动两次也不丢失)
    g_log.fp = fopen(g_log.file_path, "a");
    if (!g_log.fp) {
        fprintf(stderr, "[LOG] WARNING: Cannot open log file '%s': %s\n",
                g_log.file_path, strerror(errno));
        fprintf(stderr, "[LOG]         Logging to stderr only\n");
    } else {
        // 行缓冲: 每行自动 flush, 兼顾性能和崩溃安全
        setvbuf(g_log.fp, NULL, _IOLBF, 0);

        // 写入日志文件头
        char ts[32];
        timestamp_str(ts, sizeof(ts));
        fprintf(g_log.fp, "==============================================================\n");
        fprintf(g_log.fp, "  SGDMADDR3v0 Host Log  --  %s\n", ts);
        fprintf(g_log.fp, "  PID: %d\n", (int)getpid());
        fprintf(g_log.fp, "==============================================================\n\n");
    }

    g_log.level     = level;
    g_log.initialized = 1;

    // 检查是否是终端 (管道/重定向时不输出颜色)
    if (!isatty(fileno(stderr))) {
        g_log.color_enabled = 0;
    }

    return (g_log.fp != NULL) ? 0 : -1;
}

void sgdma_log_set_level(SgdmaLogLevel level) {
    if (level > SGDMA_LOG_TRACE) level = SGDMA_LOG_TRACE;
    g_log.level = level;
}

SgdmaLogLevel sgdma_log_get_level(void) {
    return g_log.level;
}

const char *sgdma_log_file_path(void) {
    return g_log.file_path[0] ? g_log.file_path : NULL;
}

void sgdma_log_write(SgdmaLogLevel level, const char *file, int line,
                     const char *func, const char *fmt, ...) {
    // 级别过滤
    if (level > g_log.level) return;
    if (level < 0 || level >= SGDMA_LOG_LEVEL_COUNT) return;

    const char *tag   = g_level_meta[level].tag;
    const char *color = g_level_meta[level].color;

    // 时间戳
    char ts[32];
    timestamp_str(ts, sizeof(ts));

    // 文件名 (仅基名, 保持简洁)
    const char *fname = file ? basename_s(file) : "???";
    const char *f = func ? func : "???";

    // ---- 格式化消息 ----
    va_list args;
    va_start(args, fmt);
    char msg_buf[4096];
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);

    // ---- 写入控制台 (stderr) ----
    if (g_log.color_enabled) {
        fprintf(stderr, "%s%s %s%-5s%s %s:%d %s%-20s%s  %s\n",
                COLOR_GRAY, ts, COLOR_BOLD, tag, COLOR_RESET,
                fname, line,
                COLOR_GRAY, f, COLOR_RESET,
                msg_buf);
    } else {
        fprintf(stderr, "%s %-5s %s:%d [%s]  %s\n",
                ts, tag, fname, line, f, msg_buf);
    }
    fflush(stderr);

    // ---- 写入日志文件 ----
    if (g_log.fp) {
        fprintf(g_log.fp, "%s %-5s %s:%d [%s]  %s\n",
                ts, tag, fname, line, f, msg_buf);
        // _IOLBF 行缓冲, 换行自动 flush, 但保险起见:
        // fflush(g_log.fp);  // 如需强一致性可取消注释
    }
}

void sgdma_log_hexdump(SgdmaLogLevel level, const char *label,
                       const void *data, size_t len) {
    if (level > g_log.level) return;
    if (!data || len == 0) return;

    const uint8_t *bytes = (const uint8_t *)data;
    const char   *title = label ? label : "HEXDUMP";

    sgdma_log_write(level, "", 0, "", "▼ %s (%zu bytes):", title, len);

    char line_buf[128]; // 一行: offset + hex + ascii
    for (size_t off = 0; off < len; off += 16) {
        // offset
        int pos = snprintf(line_buf, sizeof(line_buf), "  %08zx  ", off);

        // hex (两个 byte 一组, 中间空格)
        for (size_t j = 0; j < 16; j++) {
            if (j == 8) pos += snprintf(line_buf + pos, sizeof(line_buf) - pos, " ");
            if (off + j < len) {
                pos += snprintf(line_buf + pos, sizeof(line_buf) - pos,
                                "%02x ", bytes[off + j]);
            } else {
                pos += snprintf(line_buf + pos, sizeof(line_buf) - pos, "   ");
            }
        }

        // ascii dump
        pos += snprintf(line_buf + pos, sizeof(line_buf) - pos, " |");
        for (size_t j = 0; j < 16 && off + j < len; j++) {
            uint8_t c = bytes[off + j];
            pos += snprintf(line_buf + pos, sizeof(line_buf) - pos,
                            "%c", (c >= 32 && c < 127) ? c : '.');
        }
        snprintf(line_buf + pos, sizeof(line_buf) - pos, "|");

        // 直接输出, 跳过 sgdma_log_write 以避免添加时间戳前缀 (格式会乱)
        if (g_log.color_enabled) {
            fprintf(stderr, "%s%s%s\n", COLOR_GRAY, line_buf, COLOR_RESET);
        } else {
            fprintf(stderr, "%s\n", line_buf);
        }
        if (g_log.fp) {
            fprintf(g_log.fp, "%s\n", line_buf);
        }
    }
}

void sgdma_log_flush(void) {
    if (g_log.fp) fflush(g_log.fp);
}

void sgdma_log_close(void) {
    if (g_log.initialized && g_log.fp) {
        char ts[32];
        timestamp_str(ts, sizeof(ts));
        fprintf(g_log.fp, "\n-- Log closed at %s --\n", ts);
        fflush(g_log.fp);
        fclose(g_log.fp);
        g_log.fp = NULL;
    }
    g_log.initialized = 0;
}
