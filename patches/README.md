# FFmpeg HEVC WPP 并行解码补丁

## 基于版本

FFmpeg commit: `3ab8b976c1` (avformat/matroskaenc: parse Opus packets to write proper durations)

## 补丁说明

`0001-hevc-wpp-parallel-decode.patch` - HEVC WPP (Wavefront Parallel Processing) 并行解码实现

### 修改的文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `libavcodec/hevc/thread.h` | 新增 | WPP 线程数据结构 |
| `libavcodec/hevc/thread.c` | 新增 | Score-Based 任务调度 |
| `libavcodec/hevc/hevcdec.h` | 修改 | executor 集成 |
| `libavcodec/hevc/hevcdec.c` | 修改 | WPP 路径选择 |
| `libavcodec/hevc/cabac.c` | 修改 | CABAC 状态传递 |
| `libavcodec/hevc/filter.c` | 修改 | 滤波函数拆分 |
| `libavcodec/hevc/refs.c` | 修改 | 头文件路径 |
| `libavcodec/hevc/Makefile` | 修改 | 添加 thread.o |

## 使用方法

```bash
# 1. 克隆 FFmpeg
git clone https://github.com/FFmpeg/FFmpeg.git
cd FFmpeg
git checkout 3ab8b976c1  # 切换到对应版本

# 2. 应用补丁
git apply /path/to/0001-hevc-wpp-parallel-decode.patch

# 3. 编译
./configure --enable-gpl
make -j$(nproc)
```
