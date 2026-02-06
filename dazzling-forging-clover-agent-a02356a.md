# HEVC WPP 并行解码架构设计

## 1. 设计概述

### 1.1 设计目标

实现不依赖码流语法 (entry_point_offset) 的帧内 WPP 多线程并行解码，参考 VVC 的 Score-Based 依赖管理机制，复用 FFExecutor 任务执行器，实现 CTU 级任务粒度，并将解码与滤波阶段分离。

### 1.2 核心设计理念

```
VVC 方案的核心优势:
1. Score-Based 依赖管理: 原子计数器跟踪依赖，达标即执行
2. 多阶段流水线: 解码与滤波完全分离
3. FFExecutor 复用: 统一的多优先级任务队列
4. CTU 级粒度: 比行级更细的并行度

HEVC 现状问题:
1. 依赖 entry_point_offset 语法 (不是所有码流都有)
2. 解码与滤波紧耦合 (在同一个函数中执行)
3. 使用 execute2() 行级并行 (粒度较粗)
4. 简单的行进度计数 (不够灵活)
```

## 2. 阶段流水线设计

### 2.1 任务阶段定义

```c
typedef enum HEVCTaskStage {
    HEVC_TASK_STAGE_PARSE,       // 熵解码 + CABAC
    HEVC_TASK_STAGE_INTER,       // 帧间预测 (等待参考帧)
    HEVC_TASK_STAGE_RECON,       // 重建 (帧内预测 + 残差)
    HEVC_TASK_STAGE_DEBLOCK_V,   // 垂直 Deblocking
    HEVC_TASK_STAGE_DEBLOCK_H,   // 水平 Deblocking
    HEVC_TASK_STAGE_SAO,         // SAO 滤波
    HEVC_TASK_STAGE_LAST
} HEVCTaskStage;
```

### 2.2 阶段依赖关系图

```
PARSE 阶段依赖:
┌────────────────────────────────────────────────────────┐
│                     WPP CABAC 依赖                      │
│  CTU(x,y) 需要等待:                                     │
│  - CTU(x-1, y) PARSE 完成 (左邻 CABAC 上下文)           │
│  - CTU(x+1, y-1) PARSE 完成 (右上邻 WPP 状态保存)       │
│  - 仅行首: CTU(1, y-1) PARSE 完成 (CABAC 状态复制)      │
└────────────────────────────────────────────────────────┘

     ┌───┬───┬───┬───┬───┬───┐
y-1: │   │ C │ P │   │   │   │   C = CABAC 状态来源 (x=1)
     ├───┼───┼───┼───┼───┼───┤   P = 上一行已解析
y:   │ L │ * │   │   │   │   │   L = 左邻 (必须完成)
     └───┴───┴───┴───┴───┴───┘   * = 当前 CTU

INTER 阶段依赖:
- 参考帧对应位置像素就绪 (通过 ProgressListener 机制)
- PARSE 阶段完成

RECON 阶段依赖:
┌───┬───┬───┐
│   │ T │   │   T = 上方 CTU RECON 完成
├───┼───┼───┤   L = 左侧 CTU RECON 完成
│ L │ * │   │   * = 当前 CTU
└───┴───┴───┘

DEBLOCK_V 阶段依赖:
┌───┬───┐
│ L │ * │   L = 左侧 CTU DEBLOCK_V 完成 (处理 CTU 左边界)
└───┴───┘

DEBLOCK_H 阶段依赖:
┌───┬───┐
│ T │   │   T = 上方 CTU DEBLOCK_H 完成
├───┼───┤   L = 左侧 CTU DEBLOCK_V 完成
│ L │ * │
└───┴───┘

SAO 阶段依赖:
┌───┬───┬───┐
│LT │ T │RT │   需要周围 5 个 CTU 的 DEBLOCK_H 完成:
├───┼───┼───┤   LT, T, RT, L, LB
│ L │ * │   │
├───┼───┼───┤
│LB │   │   │
└───┴───┴───┘
```

### 2.3 目标分数表

```c
// 各阶段需要的目标分数 (score 达到此值时可调度)
static const uint8_t target_score[] = {
    2,  // HEVC_TASK_STAGE_PARSE:     left + (wpp ? above_right : 1)
    0,  // HEVC_TASK_STAGE_INTER:     动态计算 (参考帧数量)
    2,  // HEVC_TASK_STAGE_RECON:     left + top
    1,  // HEVC_TASK_STAGE_DEBLOCK_V: left
    2,  // HEVC_TASK_STAGE_DEBLOCK_H: left_v + top_h
    5,  // HEVC_TASK_STAGE_SAO:       5 个邻居
};
```

## 3. 核心数据结构设计

### 3.1 HEVCTask 任务结构

```c
// 文件: libavcodec/hevc/thread.h

typedef struct HEVCProgressListener {
    struct HEVCProgressListener *next;
    struct HEVCTask *task;
    HEVCContext *s;
    int y;                              // 等待的 y 坐标
    void (*progress_done)(struct HEVCProgressListener *l);
} HEVCProgressListener;

typedef struct HEVCTask {
    union {
        struct HEVCTask *next;          // 用于 executor 调试
        FFTask task;                    // 继承 FFTask
    } u;

    HEVCTaskStage stage;                // 当前阶段

    int rx, ry;                         // CTU 坐标 (x, y in CTU units)
    int rs;                             // CTU 光栅扫描地址

    HEVCFrameContext *fc;               // 帧上下文

    // PARSE 阶段专用
    SliceHeader *sh;                    // 当前 slice header
    int ctu_idx;                        // slice 内 CTU 索引

    // 帧间预测依赖监听器
    HEVCProgressListener listener[2][HEVC_MAX_REFS];

    // Score-Based 调度
    atomic_uchar score[HEVC_TASK_STAGE_LAST];
    atomic_uchar target_inter_score;    // 动态 INTER 阶段目标分数
} HEVCTask;
```

### 3.2 HEVCRowThread 行线程状态

```c
typedef struct HEVCRowThread {
    atomic_int col_progress[HEVC_PROGRESS_LAST];  // 各阶段列进度
} HEVCRowThread;

typedef enum HEVCProgress {
    HEVC_PROGRESS_MV,      // 运动向量就绪
    HEVC_PROGRESS_PIXEL,   // 像素就绪
    HEVC_PROGRESS_LAST
} HEVCProgress;
```

### 3.3 HEVCFrameThread 帧线程上下文

```c
typedef struct HEVCFrameThread {
    atomic_int ret;                     // 错误返回值

    HEVCRowThread *rows;                // 行状态数组
    HEVCTask *tasks;                    // CTU 任务数组

    int ctb_size;                       // CTU 大小
    int ctb_width;                      // CTU 列数
    int ctb_height;                     // CTU 行数
    int ctb_count;                      // CTU 总数

    // 调度状态
    atomic_int nb_scheduled_tasks;
    atomic_int nb_scheduled_listeners;

    // 行进度 (用于帧间进度报告)
    int row_progress[HEVC_PROGRESS_LAST];

    AVMutex lock;
    AVCond cond;
} HEVCFrameThread;
```

### 3.4 CABAC WPP 状态管理

```c
// 保留现有 HEVCCABACState 结构
typedef struct HEVCCABACState {
    uint8_t state[HEVC_CONTEXTS];       // CABAC 上下文状态
    uint8_t stat_coeff[HEVC_STAT_COEFFS];
} HEVCCABACState;

// WPP 状态传递
// - 每行首 CTU(0, y) 解析前，从 CTU(1, y-1) 复制 CABAC 状态
// - CTU(1, y) 解析后保存 CABAC 状态供下一行使用
```

## 4. 文件结构

### 4.1 新增文件

```
libavcodec/hevc/
├── thread.c          # 新增: WPP 线程管理实现
├── thread.h          # 新增: WPP 线程数据结构定义
└── Makefile          # 修改: 添加 thread.o
```

### 4.2 修改文件

```
libavcodec/hevc/
├── hevcdec.h         # 修改: 添加 HEVCFrameThread 指针
├── hevcdec.c         # 修改: 集成新 WPP 机制
├── filter.c          # 修改: 支持分离的滤波阶段
├── cabac.c           # 修改: 支持 WPP CABAC 状态保存/恢复

libavcodec/
├── executor.c        # 复用: 不需要修改
├── executor.h        # 复用: 不需要修改
```

## 5. 核心函数设计

### 5.1 任务调度函数

```c
// thread.c

// 初始化帧线程上下文
int ff_hevc_frame_thread_init(HEVCFrameContext *fc);

// 释放帧线程上下文
void ff_hevc_frame_thread_free(HEVCFrameContext *fc);

// 提交帧进行解码
int ff_hevc_frame_submit(HEVCContext *s, HEVCFrameContext *fc);

// 等待帧解码完成
int ff_hevc_frame_wait(HEVCContext *s, HEVCFrameContext *fc);

// 创建 executor
FFExecutor* ff_hevc_executor_alloc(HEVCContext *s, int thread_count);
void ff_hevc_executor_free(FFExecutor **e);

// 内部函数
static void add_task(HEVCContext *s, HEVCTask *t);
static void frame_thread_add_score(HEVCContext *s, HEVCFrameThread *ft,
                                   int rx, int ry, HEVCTaskStage stage);
static int task_has_target_score(HEVCTask *t, HEVCTaskStage stage, uint8_t score);
```

### 5.2 各阶段运行函数

```c
// thread.c

static int run_parse(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
static int run_inter(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
static int run_recon(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
static int run_deblock_v(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
static int run_deblock_h(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
static int run_sao(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t);
```

### 5.3 CABAC WPP 状态传递

```c
// cabac.c 修改

// 保存 CABAC 状态 (CTU(1, y) 解析后调用)
void ff_hevc_save_cabac_state_wpp(HEVCLocalContext *lc, int ctb_addr_ts);

// 恢复 CABAC 状态 (CTU(0, y) 解析前调用)
void ff_hevc_restore_cabac_state_wpp(HEVCLocalContext *lc,
                                      const HEVCCABACState *src);

// 初始化 WPP 下一行 CABAC 状态
static void ep_init_wpp(HEVCLocalContext *next,
                         const HEVCLocalContext *current);
```

### 5.4 进度报告函数

```c
// thread.c

// 报告帧进度 (用于帧间依赖)
static void report_frame_progress(HEVCFrameContext *fc,
                                   int ry, HEVCProgress idx);

// 添加进度监听器
static void add_progress_listener(HEVCFrame *ref, HEVCProgressListener *l,
                                   HEVCTask *t, HEVCContext *s,
                                   HEVCProgress vp, int y);
```

## 6. 执行流程

### 6.1 帧解码主流程

```
1. ff_hevc_frame_submit()
   ├── 遍历所有 slice
   │   └── 遍历所有 CTU
   │       ├── 初始化 HEVCTask
   │       └── 关联 slice 和 CTU 索引
   ├── 初始化边界分数
   └── 提交 CTU(0,0) 到 INIT 阶段

2. INIT 阶段完成后
   ├── 检查帧间依赖 (collocated frame)
   └── 提交所有 entry point 首 CTU

3. 各 CTU 按 score 调度执行
   ├── PARSE → INTER → RECON → DEBLOCK_V → DEBLOCK_H → SAO
   └── 每阶段完成后更新邻居分数

4. ff_hevc_frame_wait()
   ├── 等待所有任务完成
   └── 返回解码结果
```

### 6.2 任务执行流程

```
task_run():
    do {
        task_run_stage(t, s, lc);  // 执行当前阶段
        t->stage++;                 // 进入下一阶段
    } while (task_is_stage_ready(t, 1));  // 检查是否可继续

    if (t->stage != HEVC_TASK_STAGE_LAST)
        frame_thread_add_score(s, ft, t->rx, t->ry, t->stage);  // 更新分数
```

### 6.3 阶段完成后分数更新

```c
static void task_stage_done(const HEVCTask *t, HEVCContext *s) {
    switch (t->stage) {
    case HEVC_TASK_STAGE_PARSE:
        // 通知右邻和下邻可以开始 PARSE
        ADD(1, 0, HEVC_TASK_STAGE_PARSE);   // right
        ADD(0, 1, HEVC_TASK_STAGE_PARSE);   // below (WPP)
        ADD(-1, 1, HEVC_TASK_STAGE_PARSE);  // below-left (WPP)
        break;

    case HEVC_TASK_STAGE_RECON:
        ADD(1, 0, HEVC_TASK_STAGE_RECON);   // right
        ADD(0, 1, HEVC_TASK_STAGE_RECON);   // below
        break;

    case HEVC_TASK_STAGE_DEBLOCK_V:
        ADD(1, 0, HEVC_TASK_STAGE_DEBLOCK_V);
        ADD(1, 0, HEVC_TASK_STAGE_DEBLOCK_H);
        break;

    case HEVC_TASK_STAGE_DEBLOCK_H:
        ADD(0, 1, HEVC_TASK_STAGE_DEBLOCK_H);
        // 通知 SAO 依赖
        ADD(-1, -1, HEVC_TASK_STAGE_SAO);
        ADD(0, -1, HEVC_TASK_STAGE_SAO);
        ADD(1, -1, HEVC_TASK_STAGE_SAO);
        ADD(-1, 0, HEVC_TASK_STAGE_SAO);
        ADD(-1, 1, HEVC_TASK_STAGE_SAO);
        break;
    }
}
```

## 7. WPP CABAC 状态传递机制

### 7.1 状态保存点

```
WPP 要求在每行第二个 CTU (x=1) 解析完成后保存 CABAC 状态:

Row y:   [CTU0] [CTU1] [CTU2] [CTU3] ...
                  ↓
            save_state()
                  ↓
Row y+1: [CTU0] [CTU1] [CTU2] [CTU3] ...
            ↑
       restore_state()
```

### 7.2 实现代码

```c
// 在 run_parse() 中
static int run_parse(HEVCContext *s, HEVCLocalContext *lc, HEVCTask *t) {
    // ... 解析 CTU ...

    // WPP: 保存 CTU(1, y) 的 CABAC 状态
    if (sps->entropy_coding_sync_enabled_flag && t->rx == 1) {
        ff_hevc_save_cabac_state_wpp(lc, t->rs);
    }

    // WPP: 通知下一行 CTU(0, y+1) 可以开始
    if (sps->entropy_coding_sync_enabled_flag && t->rx >= 1) {
        frame_thread_add_score(s, ft, 0, t->ry + 1, HEVC_TASK_STAGE_PARSE);
    }

    return 0;
}

// 在开始解析 CTU(0, y) 前
static void restore_wpp_state(HEVCLocalContext *lc, HEVCTask *t) {
    if (t->rx == 0 && t->ry > 0) {
        // 从上一行的共享状态恢复
        const HEVCCABACState *wpp_state = &lc->fc->wpp_cabac_state[t->ry - 1];
        ff_hevc_restore_cabac_state_wpp(lc, wpp_state);
    }
}
```

## 8. 与现有代码兼容策略

### 8.1 编译时选择

```c
// hevcdec.c

static int decode_slice_data(HEVCContext *s, ...) {
    // ...

    if (s->executor) {
        // 新的 CTU 级 WPP (不依赖 entry_point_offset)
        return ff_hevc_frame_submit(s, fc);
    } else if (s->avctx->active_thread_type == FF_THREAD_SLICE &&
               s->sh.num_entry_point_offsets > 0 &&
               pps->entropy_coding_sync_enabled_flag) {
        // 旧的行级 WPP (依赖 entry_point_offset)
        return hls_slice_data_wpp(s, nal);
    } else {
        // 单线程解码
        return hls_decode_entry(s, gb);
    }
}
```

### 8.2 运行时配置

```c
// hevcdec.c init

static av_cold int hevc_decode_init(AVCodecContext *avctx) {
    HEVCContext *s = avctx->priv_data;

    // 分配 executor (线程数 > 1 时启用)
    if (avctx->thread_count > 1) {
        s->executor = ff_hevc_executor_alloc(s, avctx->thread_count);
        if (!s->executor) {
            av_log(avctx, AV_LOG_WARNING,
                   "Failed to create executor, falling back to legacy WPP\n");
        }
    }

    return 0;
}
```

### 8.3 AVOption 开关

```c
static const AVOption hevc_options[] = {
    { "threads", "number of threads", OFFSET(threads), AV_OPT_TYPE_INT,
      { .i64 = 0 }, 0, INT_MAX, VD },
    { "thread_type", "select threading method", OFFSET(thread_type), AV_OPT_TYPE_FLAGS,
      { .i64 = FF_THREAD_FRAME | FF_THREAD_SLICE }, 0, INT_MAX, VD, "thread_type" },
    { "wpp", "use WPP parallel decoding", OFFSET(wpp_type), AV_OPT_TYPE_INT,
      { .i64 = 1 }, 0, 2, VD, "wpp_type" },
        { "auto", "automatic selection", 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 0, VD, "wpp_type" },
        { "legacy", "legacy row-level WPP", 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 0, VD, "wpp_type" },
        { "ctu", "CTU-level WPP with executor", 0, AV_OPT_TYPE_CONST, { .i64 = 2 }, 0, 0, VD, "wpp_type" },
    { NULL },
};
```

## 9. 任务优先级设计

```c
// 优先级: 0 最高, 2 最低
static const int task_priorities[] = {
    0,  // HEVC_TASK_STAGE_PARSE     - 最高优先级
    2,  // HEVC_TASK_STAGE_INTER     - 最低优先级 (避免 inter 任务堆积)
    1,  // HEVC_TASK_STAGE_RECON
    1,  // HEVC_TASK_STAGE_DEBLOCK_V
    1,  // HEVC_TASK_STAGE_DEBLOCK_H
    1,  // HEVC_TASK_STAGE_SAO
};

#define PRIORITY_LOWEST 2
```

## 10. 帧间进度报告机制

```c
// 用于 B 帧等待参考帧像素就绪

typedef struct HEVCProgressListener {
    struct HEVCProgressListener *next;
    struct HEVCTask *task;
    HEVCContext *s;
    HEVCProgress vp;     // HEVC_PROGRESS_MV 或 HEVC_PROGRESS_PIXEL
    int y;               // 等待的 y 坐标
    void (*progress_done)(struct HEVCProgressListener *l);
} HEVCProgressListener;

// 参考帧进度跟踪
typedef struct HEVCFrame {
    // ... existing fields ...
    atomic_int progress[HEVC_PROGRESS_LAST];  // 每种进度类型
    HEVCProgressListener *listeners;           // 等待此帧的任务
    AVMutex progress_lock;
} HEVCFrame;

// 报告进度
void ff_hevc_report_progress(HEVCFrame *f, HEVCProgress idx, int y) {
    ff_mutex_lock(&f->progress_lock);
    atomic_store(&f->progress[idx], y);

    // 唤醒等待的任务
    HEVCProgressListener *l = f->listeners;
    while (l) {
        if (l->vp == idx && l->y <= y) {
            l->progress_done(l);
        }
        l = l->next;
    }
    ff_mutex_unlock(&f->progress_lock);
}
```

## 11. 错误处理

```c
// 任何任务失败时设置错误标志
if ((ret = run[stage](s, lc, t)) < 0) {
    int zero = 0;
    atomic_compare_exchange_strong(&ft->ret, &zero, ret);
    // 记录错误位置
    av_log(s->avctx, AV_LOG_ERROR,
           "frame %d, %s(%d, %d) failed with %d\n",
           fc->decode_order, task_name[stage], t->rx, t->ry, ret);
}

// 等待时检查错误
int ff_hevc_frame_wait(HEVCContext *s, HEVCFrameContext *fc) {
    HEVCFrameThread *ft = fc->ft;

    ff_mutex_lock(&ft->lock);
    while (atomic_load(&ft->nb_scheduled_tasks) ||
           atomic_load(&ft->nb_scheduled_listeners))
        ff_cond_wait(&ft->cond, &ft->lock);
    ff_mutex_unlock(&ft->lock);

    return ft->ret;  // 返回第一个错误
}
```

## 12. 实现步骤

### Phase 1: 基础框架 (Task #4)
1. 创建 `libavcodec/hevc/thread.h`，定义数据结构
2. 创建 `libavcodec/hevc/thread.c`，实现基本框架
3. 集成 FFExecutor
4. 实现 PARSE 阶段

### Phase 2: 解码阶段分离 (Task #5)
1. 将 `hls_coding_quadtree` 拆分为 PARSE + RECON
2. 实现 INTER 阶段 (帧间预测)
3. 实现 RECON 阶段 (重建)

### Phase 3: 滤波阶段分离
1. 将 `ff_hevc_hls_filters` 拆分为 DEBLOCK_V + DEBLOCK_H + SAO
2. 实现边界分数初始化

### Phase 4: WPP CABAC 状态传递
1. 修改 `cabac.c` 支持状态保存/恢复
2. 实现 CTU(1, y) 保存 + CTU(0, y+1) 恢复

### Phase 5: 帧间依赖管理
1. 实现 ProgressListener 机制
2. 实现 `ff_hevc_report_progress`
3. 集成到 INTER 阶段

### Phase 6: 集成测试 (Task #6)
1. FATE 测试验证
2. 性能基准测试
3. 边界情况测试

## 13. 预期性能提升

```
场景分析:
- 4K HEVC: ~8100 CTU (120x68)
- 每行 120 个 CTU
- 传统 WPP: 最多 68 个并行任务 (行级)
- 新方案: 理论上可达 120-200 个并行任务 (CTU 级)

预期提升:
- 2-4 核: 约 10-20% (受 CABAC 串行瓶颈限制)
- 8+ 核: 约 30-50% (更好的 CPU 利用率)
- 关键: 解码/滤波分离减少等待时间
```

## 14. 关键差异总结: VVC vs HEVC

| 特性 | VVC | HEVC (新设计) |
|------|-----|---------------|
| 阶段数 | 10 | 6 |
| LMCS 阶段 | 有 | 无 |
| ALF 阶段 | 有 | 无 (HEVC 无 ALF) |
| Tiles 支持 | 完整 | 简化 (复用现有) |
| CABAC WPP | ep_init_wpp() | 类似实现 |
| 进度类型 | MV + PIXEL | MV + PIXEL |

## 15. 代码示例: 关键函数骨架

```c
// libavcodec/hevc/thread.c

#include <stdatomic.h>
#include "libavcodec/executor.h"
#include "libavutil/mem.h"
#include "libavutil/thread.h"
#include "thread.h"
#include "hevcdec.h"

static void add_task(HEVCContext *s, HEVCTask *t) {
    HEVCFrameThread *ft = t->fc->ft;
    FFTask *task = &t->u.task;

    static const int priorities[] = {
        0,  // PARSE
        2,  // INTER
        1,  // RECON
        1,  // DEBLOCK_V
        1,  // DEBLOCK_H
        1,  // SAO
    };

    atomic_fetch_add(&ft->nb_scheduled_tasks, 1);
    task->priority = priorities[t->stage];
    ff_executor_execute(s->executor, task);
}

static int task_run(FFTask *_t, void *local_context, void *user_data) {
    HEVCTask *t = (HEVCTask *)_t;
    HEVCContext *s = user_data;
    HEVCLocalContext *lc = local_context;
    HEVCFrameThread *ft = t->fc->ft;

    lc->fc = t->fc;

    do {
        task_run_stage(t, s, lc);
        t->stage++;
    } while (task_is_stage_ready(t, 1));

    if (t->stage != HEVC_TASK_STAGE_LAST)
        frame_thread_add_score(s, ft, t->rx, t->ry, t->stage);

    scheduled_done(ft, &ft->nb_scheduled_tasks);
    return 0;
}

FFExecutor* ff_hevc_executor_alloc(HEVCContext *s, int thread_count) {
    FFTaskCallbacks callbacks = {
        s,
        sizeof(HEVCLocalContext),
        PRIORITY_LOWEST + 1,
        task_run,
    };
    return ff_executor_alloc(&callbacks, thread_count);
}
```

---

**设计完成时间**: 2026-02-06
**设计者**: architect agent
**状态**: 待审核
