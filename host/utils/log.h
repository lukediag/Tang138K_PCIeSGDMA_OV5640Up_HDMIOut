// ============================================================================
// log.h — SGDMADDR3v0 日志系统头文件
// ============================================================================
// 功能:
//   - 5 级日志: ERROR / WARN / INFO / DEBUG / TRACE
//   - 自动时间戳 (精确到微秒)
//   - 控制台 (stderr) + 文件双输出
//   - 控制台 ANSI 彩色输出 (可禁用)
//   - hexdump 二进制数据转储
//   - 自动创建日志目录
//   - 文件创建失败时仅回退到 stderr, 不阻塞程序
//
// 使用:
//   sgdma_log_init("./logs", "sgdma", SGDMA_LOG_INFO);
//   SLOG_INFO("PCIe ready, BAR0=%p", bar0);
//   SLOG_ERROR("DMA timeout! poll=0x%08x", poll);
//   sgdma_log_hexdump(SGDMA_LOG_DEBUG, "descriptors", desc_mem, 256);
//   sgdma_log_close();
// ============================================================================

#ifndef SGDMA_LOG_H
#define SGDMA_LOG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 日志级别 ----
typedef enum {
    SGDMA_LOG_ERROR = 0,   // 致命错误, 程序将退出
    SGDMA_LOG_WARN  = 1,   // 警告, 可恢复
    SGDMA_LOG_INFO  = 2,   // 一般信息 (默认级别)
    SGDMA_LOG_DEBUG = 3,   // 调试详情
    SGDMA_LOG_TRACE = 4,   // 细粒度追踪 (函数入口/退出等)
    SGDMA_LOG_LEVEL_COUNT
} SgdmaLogLevel;

// ============================================================================
// 日志系统 API
// ============================================================================

// 初始化日志系统
//   log_dir: 日志文件存放目录 (NULL = ".")
//   prefix:  日志文件名前缀 (NULL = "sgdma")
//   level:   最低输出级别 (低于此级别的消息被过滤)
// 返回 0 成功, -1 文件创建失败 (回退到仅 stderr 输出, 仍可用)
int sgdma_log_init(const char *log_dir, const char *prefix, SgdmaLogLevel level);

// 动态修改日志级别
void sgdma_log_set_level(SgdmaLogLevel level);

// 获取当前日志级别
SgdmaLogLevel sgdma_log_get_level(void);

// 获取日志文件路径 (方便打印给用户看)
const char *sgdma_log_file_path(void);

// 写入一条日志
// 通常不直接调用, 使用下面的便捷宏
void sgdma_log_write(SgdmaLogLevel level, const char *file, int line,
                     const char *func, const char *fmt, ...)
    __attribute__((format(printf, 5, 6)));

// 以 hexdump 格式输出二进制数据
//   level: 日志级别
//   label: 数据标签 (NULL = "HEXDUMP")
//   data:  数据指针
//   len:   字节数
void sgdma_log_hexdump(SgdmaLogLevel level, const char *label,
                       const void *data, size_t len);

// 强制刷新日志缓冲区到磁盘
void sgdma_log_flush(void);

// 关闭日志系统 (释放资源, 关闭文件)
void sgdma_log_close(void);

// ---- 便捷宏 (自动捕获 __FILE__, __LINE__, __func__) ----
#define SLOG_ERROR(fmt, ...) \
    sgdma_log_write(SGDMA_LOG_ERROR, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define SLOG_WARN(fmt, ...) \
    sgdma_log_write(SGDMA_LOG_WARN,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define SLOG_INFO(fmt, ...) \
    sgdma_log_write(SGDMA_LOG_INFO,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define SLOG_DEBUG(fmt, ...) \
    sgdma_log_write(SGDMA_LOG_DEBUG, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define SLOG_TRACE(fmt, ...) \
    sgdma_log_write(SGDMA_LOG_TRACE, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

// hexdump 便捷宏
#define SLOG_HEXDUMP(level, label, data, len) \
    sgdma_log_hexdump(level, label, data, len)

#ifdef __cplusplus
}
#endif

#endif // SGDMA_LOG_H
