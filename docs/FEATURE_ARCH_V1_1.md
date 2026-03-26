# VectorFOC v1.1 功能架构设计

> 状态：**草案（待评审）**
> 作者：穹沛科技 · VectorFOC 团队
> 版本：v1.1.0-draft
> 日期：2026-03-12

---

## 概述

v1.0 发布后遗留两个非阻塞性缺口，本文档定义其 v1.1 的功能架构与实现规范。

| 特性 | 文件影响范围 | 优先级 |
|------|------------|--------|
| **F1** CAN 总线扫描（多节点拓扑发现） | `bsp_can`, `inovxio_protocol`, `manager` | P1 — Thunder 多电机必要 |
| **F2** Flash 参数原子写入 | `param_storage`, `bsp_flash` | P2 — 电源保护增强 |

---

## F1：CAN 总线扫描（Bus Scan）

### 1.1 问题背景

当前 `PRIVATE_CMD_GET_ID`（CMD 0）支持广播寻址（Target = `0x7F`），每个节点收到广播后会回复自身 ID。
但现有 **主机侧**（Brainstem / OpenClaw Console）没有对应的"发广播 → 收集多帧回复 → 建立节点表"流程，`scan_bus` 实际上只返回本机 ID。

**Thunder 四足机器狗场景**：12 个关节电机挂在同一 CAN 总线，主机启动时需要自动发现哪些节点在线，避免每次写死 ID 列表。

### 1.2 设计目标

- 主机发 1 帧广播 → 收集 ≤ 127 个节点的回复（窗口期 20 ms）
- 生成 `NodeTable`（CAN ID → UID 映射），供上层使用
- 不阻塞实时控制任务
- 不引入新 CAN 命令码（复用 CMD 0）

### 1.3 协议层（无需新增命令）

CMD 0 已完整支持广播查询，节点回复帧格式：

```
Arbitration ID: [28:24]=0x00 | [23:8]=0x0000 | [7:0]=0xFD (→ 主机)
DLC: 8
Data[0..3]: STM32 UID 低 4 字节
Data[4]:    CAN ID (本节点当前配置值)
Data[5..7]: Reserved / UID[4..6]
```

**广播帧（主机发）**：

```
Arbitration ID: [28:24]=0x00 | [23:8]=0x0000 | [7:0]=0x7F
DLC: 0
```

### 1.4 固件侧变更

固件 **无需改动**。CMD 0 广播响应逻辑已在 `inovxio_protocol.c` 中实现：收到 Target=`0x7F` 的 CMD 0 时回复自身信息。

### 1.5 主机侧（嵌入式 / PC）架构

```
┌─────────────────────────────────────────────────────────┐
│                   scan_bus() API                        │
│  输入：timeout_ms (建议 20ms)                            │
│  输出：NodeEntry[] + count                               │
└────────────────┬────────────────────────────────────────┘
                 │
    ┌────────────▼────────────┐
    │  1. 停止自动上报收集      │  (pause FIFO handler)
    │  2. 发送广播帧 CMD 0     │  Target=0x7F, DLC=0
    │  3. 等待 timeout_ms      │  HAL_GetTick() 轮询
    │  4. 收集所有回复          │  过滤 CMD=0, Target=0xFD
    │  5. 解析回复 → NodeEntry  │  提取 CAN_ID + UID
    │  6. 恢复自动上报收集      │
    └────────────┬────────────┘
                 │
    ┌────────────▼────────────┐
    │       NodeTable         │
    │  uint8_t  can_id[127]   │
    │  uint32_t uid_low[127]  │
    │  uint8_t  count         │
    └─────────────────────────┘
```

### 1.6 数据结构

```c
/* 单节点条目 */
typedef struct {
    uint8_t  can_id;        /* 节点 CAN ID (1-126) */
    uint32_t uid_low;       /* STM32 UID 低 4 字节（唯一性标识）*/
    uint8_t  uid_extra[3];  /* UID[4..6]（可选扩展）*/
} NodeEntry_t;

/* 总线扫描结果 */
typedef struct {
    NodeEntry_t nodes[126]; /* 最多 126 个节点 (ID 1-126) */
    uint8_t     count;      /* 实际发现节点数 */
    uint32_t    scan_time_ms; /* 实际耗时 */
} NodeTable_t;
```

### 1.7 主机侧实现伪代码

```c
bool scan_bus(NodeTable_t *table, uint32_t timeout_ms) {
    memset(table, 0, sizeof(NodeTable_t));
    uint32_t t0 = HAL_GetTick();

    /* 发广播 */
    CAN_Frame req = { .id = BUILD_ID(CMD_GET_ID, 0x0000, 0x7F),
                      .dlc = 0 };
    BSP_CAN_SendFrame(&req);

    /* 收集回复 */
    while ((HAL_GetTick() - t0) < timeout_ms) {
        CAN_Frame resp;
        if (!CAN_TryDequeue(&resp)) continue;

        uint8_t cmd = GET_CMD_TYPE(resp.id);
        uint8_t tgt = GET_TARGET_ID(resp.id);
        if (cmd != CMD_GET_ID || tgt != HOST_ID) continue;

        uint8_t node_can_id = resp.data[4];
        /* 去重 */
        bool dup = false;
        for (int i = 0; i < table->count; i++) {
            if (table->nodes[i].can_id == node_can_id) { dup = true; break; }
        }
        if (!dup && table->count < 126) {
            NodeEntry_t *e = &table->nodes[table->count++];
            e->can_id = node_can_id;
            memcpy(&e->uid_low, resp.data, 4);
            memcpy(e->uid_extra, resp.data + 5, 3);
        }
    }
    table->scan_time_ms = HAL_GetTick() - t0;
    return (table->count > 0);
}
```

### 1.8 ID 碰撞处理

扫描结果如出现两个节点报告相同 `can_id`（但 `uid_low` 不同），标记为冲突：

```c
typedef enum {
    NODE_OK = 0,
    NODE_ID_COLLISION = 1,  /* 同 CAN ID，不同 UID */
} NodeStatus_t;
```

上层应在 ID 碰撞时告警，引导用户通过 CMD 7 重新分配 ID。

### 1.9 集成点

| 集成位置 | 说明 |
|---------|------|
| Brainstem `can_manager.dart` | 启动序列中调用 scan_bus，填充节点路由表 |
| OpenClaw Console | 拓扑发现页面显示 NodeTable |
| LingTu 驱动层 | 可选：初始化时验证预期节点 ID 是否在线 |

---

## F2：Flash 参数原子写入

### 2.1 问题背景

当前 `ParamStorage_Save` 顺序写两页：

```
Step 1: Erase Page1 → Write Page1 → Verify Page1
Step 2: Erase Page2 → Write Page2
```

若在 Step 1 完成、Step 2 开始前掉电，Page1 已更新而 Page2 是旧数据。
`ParamStorage_Load` 的回退逻辑只检查 `magic` 字段——恰好旧 Page2 的 magic 有效，加载的却是旧参数（静默降级）。

**场景：**
- 调机时 `save_params` 后立即掉电
- 重启后加载了掉电前的旧参数
- 机器人行为异常，调试困难

### 2.2 设计目标

- **写入原子性保证**：写失败（掉电）后，启动时始终能加载一份完整有效的参数集
- **无额外 Flash 页占用**：在现有 Page62 + Page63（共 4 KB）内实现
- **向后兼容**：旧固件烧录的参数格式（magic=`FOC1`, version）正常迁移
- **不增加写入耗时**（仍为两次 erase+write，但顺序有意义）

### 2.3 核心方案：Generation Counter + Commit Flag

在 `FlashParamData` 头部引入 **Generation** 字段，写入逻辑改为"写新页 → 验证 → 标记旧页无效"。

#### 2.3.1 页角色动态交替（Ping-Pong）

```
写第 N 次：
  - 当前 Active Page  = Page A（generation = N-1）
  - 当前 Standby Page = Page B

步骤：
  1. 构造新 FlashParamData，generation = N, committed = 0
  2. Erase Page B
  3. Write Page B（含 CRC32，committed = 0）
  4. Verify Page B
  5. 写入 committed = 1 到 Page B（单独一次 double-word 写，不需要 erase）
  6. 逻辑上 Page B 成为 Active，Page A 成为 Standby
```

**掉电恢复逻辑（Load）**：

```
读 Page A 和 Page B 的 header:
  - magic 有效 + CRC 通过 + committed == 1 → 有效页
  - 两页均有效 → 选 generation 较大的
  - 只有一页有效 → 使用该页
  - 均无效 → 返回 ERR_CORRUPT，使用默认参数
```

#### 2.3.2 FlashParamData 新头部布局

```c
typedef struct {
    uint32_t magic;       /* 0x464F4332 "FOC2"（区分旧格式）*/
    uint32_t version;     /* param schema version */
    uint32_t generation;  /* 单调递增写入计数（新增）*/
    uint32_t committed;   /* 0 = 写入中, 1 = 写入完成（新增）*/
    uint32_t crc32;       /* CRC32（对 reserved_data 之前全部字段之后的数据）*/
    uint32_t reserved;    /* 对齐 */
    /* ... 其余参数字段不变 ... */
} __attribute__((packed)) FlashParamData_v2;
```

> **注意**：magic 从 `FOC1`(`0x464F4331`) 改为 `FOC2`(`0x464F4332`)，
> Load 时同时识别两个 magic，`FOC1` 格式自动当作 generation=0 迁移。

#### 2.3.3 committed 字段的写入方案

STM32G4 Flash 最小写入单位为 64-bit double-word，一旦写 0 不可在不 erase 的情况下改回 1。
利用这个特性：

```
Page B 写入顺序：
  ① Erase Page B（全 0xFF）
  ② 写整页数据，committed 字段位置填 0x00000000（表示"写入中"）
  ③ 验证整页 CRC 通过后，将 committed 所在的 double-word 重新写为
     0x0000000100000001（高 32 位为 1 表示 committed，低 32 位为 generation）
     → 一次 HAL_FLASH_Program 调用，不需要 erase
```

**为什么不需要 erase**：
Flash 写入只能将 bit 从 1 改为 0，`committed = 0x00000000` → `0x00000001` 是合法的单次写。
若 ③ 步掉电：committed 仍为 0，Load 时该页被判定为无效，回退到 Page A（旧参数）。

#### 2.3.4 CRC 覆盖范围

```
CRC32 = CRC(data[offsetof(reserved)+4 ... end of FlashParamData])
      （跳过 magic / version / generation / committed / crc32 / reserved 共 24 字节）
```

与 v1.0 保持相同的 CRC 起点（`+16` 字节），加上新增的 8 字节（generation + committed），起点改为 `+24`。

### 2.4 新的 Save 流程

```c
FlashStorageResult ParamStorage_Save_v2(const FlashParamData_v2 *data) {
    // 1. 确定写哪页（轮换）
    uint8_t active_page  = ParamStorage_GetActivePage();  // 0=Page1, 1=Page2
    uint8_t standby_page = 1 - active_page;
    uint32_t write_addr  = standby_page ? PAGE2_ADDR : PAGE1_ADDR;

    // 2. 构造写入数据
    FlashParamData_v2 wr = *data;
    wr.magic      = FLASH_MAGIC_WORD_V2;
    wr.version    = FLASH_PARAM_VERSION;
    wr.generation = s_generation + 1;
    wr.committed  = 0;  // 写入中标志
    wr.crc32      = 0;
    wr.crc32      = BSP_Flash_CalculateCRC32(crc_start(&wr), crc_len);

    BSP_Flash_Unlock();

    // 3. Erase standby page
    if (!BSP_Flash_ErasePage(write_addr)) goto fail;

    // 4. Write full page (committed = 0)
    if (!Flash_Write(write_addr, (uint8_t*)&wr, sizeof(wr))) goto fail;

    // 5. Verify CRC
    if (!BSP_Flash_Verify(write_addr, (uint8_t*)&wr, sizeof(wr))) goto fail;

    // 6. Write committed flag (single double-word, no erase needed)
    uint32_t committed_offset = offsetof(FlashParamData_v2, committed);
    uint64_t commit_dw = ((uint64_t)1 << 32) | (uint64_t)wr.generation;
    if (!BSP_Flash_WriteDoubleWord(write_addr + committed_offset, commit_dw)) goto fail;

    BSP_Flash_Lock();
    s_generation++;
    s_active_page = standby_page;
    return FLASH_STORAGE_OK;

fail:
    BSP_Flash_Lock();
    return FLASH_STORAGE_ERR_WRITE;
}
```

### 2.5 新的 Load 流程

```c
FlashStorageResult ParamStorage_Load_v2(FlashParamData_v2 *data) {
    FlashParamData_v2 p1, p2;
    bool p1_valid = page_is_valid(PAGE1_ADDR, &p1);
    bool p2_valid = page_is_valid(PAGE2_ADDR, &p2);

    if (!p1_valid && !p2_valid) return FLASH_STORAGE_ERR_CRC;

    FlashParamData_v2 *best;
    if      (p1_valid && !p2_valid) best = &p1;
    else if (!p1_valid && p2_valid) best = &p2;
    else    best = (p1.generation >= p2.generation) ? &p1 : &p2;

    *data = *best;
    s_generation = best->generation;
    s_active_page = (best == &p1) ? 0 : 1;
    return FLASH_STORAGE_OK;
}

static bool page_is_valid(uint32_t addr, FlashParamData_v2 *out) {
    BSP_Flash_Read(addr, (uint8_t*)out, sizeof(FlashParamData_v2));
    if (out->magic != FLASH_MAGIC_WORD_V2) {
        /* 向后兼容：尝试 FOC1 格式 */
        return try_migrate_v1(addr, out);
    }
    if (out->committed != 1) return false;  /* 写入中断 */
    /* CRC 验证 */
    uint32_t stored = out->crc32;
    out->crc32 = 0;
    bool ok = (BSP_Flash_CalculateCRC32(crc_start(out), crc_len) == stored);
    out->crc32 = stored;
    return ok;
}
```

### 2.6 向后兼容迁移

```c
static bool try_migrate_v1(uint32_t addr, FlashParamData_v2 *out) {
    FlashParamData v1;
    BSP_Flash_Read(addr, (uint8_t*)&v1, sizeof(v1));
    if (v1.magic != FLASH_MAGIC_WORD_V1) return false;
    /* CRC 校验（v1 方式）*/
    if (!v1_crc_ok(&v1)) return false;
    /* 迁移：把 v1 字段复制到 v2 结构 */
    memset(out, 0, sizeof(*out));
    out->magic      = FLASH_MAGIC_WORD_V2;
    out->version    = v1.version;
    out->generation = 0;
    out->committed  = 1;  /* 迁移视为已提交 */
    /* 复制参数字段（偏移相同，reserved_data 截断无妨）*/
    memcpy(&out->motor_rs, &v1.motor_rs,
           sizeof(FlashParamData) - 16);
    return true;
}
```

### 2.7 Flash 布局（无变化）

```
0x0801F000  Page 62 (2KB)  — 未使用（保留给 OTA Bootloader）
0x0801F800  Page 63 上半 (2KB) = FLASH_PARAM_PAGE1  ← 角色动态交替
0x0801FC00  Page 63 下半 (2KB) = FLASH_PARAM_PAGE2  ← 角色动态交替
```

> `FlashParamData_v2` 大小 = `FlashParamData` 大小 + 8 字节（generation + committed）
> 调整 `reserved_data` 长度 -8 以保持结构体总大小不变（≤ 2048 字节）。

---

## 实施计划

### F1 CAN 总线扫描

| 步骤 | 位置 | 工作量 |
|------|------|--------|
| 1. 固件侧：确认 CMD 0 广播响应已覆盖所有协议模式 | `inovxio_protocol.c`, `canopen_protocol.c` | 0.5h |
| 2. 主机侧：实现 `scan_bus()` API | Brainstem `can_manager.dart` 或独立 C 库 | 2h |
| 3. 主机侧：NodeTable 数据结构 + 去重 + ID 碰撞检测 | 同上 | 1h |
| 4. OpenClaw Console：拓扑发现 UI | Flutter | 2h |
| **合计** | | **~5h** |

### F2 Flash 原子写入

| 步骤 | 位置 | 工作量 |
|------|------|--------|
| 1. `param_storage.h`：FlashParamData_v2 结构定义 | `param_storage.h` | 1h |
| 2. `param_storage.c`：Save_v2 / Load_v2 / migrate_v1 | `param_storage.c` | 3h |
| 3. `bsp_flash.c`：确认 WriteDoubleWord 可在已擦除页上追加写 committed | `bsp_flash.c` | 0.5h |
| 4. 单元测试：掉电场景模拟（RAM 模拟 Flash） | `tests/test_param_storage.c` | 2h |
| **合计** | | **~6.5h** |

**总工作量估算：~12h（1.5 个工作日）**
不阻断 v1.0 发布，可在 v1.0 tag 之后立即开始。

---

## 测试矩阵

### F1

| 场景 | 期望结果 |
|------|---------|
| 1 个节点在线 | count=1，can_id 正确 |
| 12 个节点在线（Thunder 全电机）| count=12，无丢帧 |
| 2 个节点 ID 相同（错误配置）| count=2 但标记 NODE_ID_COLLISION |
| 总线空（无节点）| count=0，超时返回 false |
| timeout=5ms（过短）| 可能漏节点，上层警告 |

### F2

| 场景 | 期望结果 |
|------|---------|
| 正常保存 + 重启 | 加载成功，参数一致 |
| 掉电在 Step 4（write）完成前 | Load 回退到旧页，参数为掉电前旧值 |
| 掉电在 Step 5（verify）完成前，数据损坏 | CRC 失败 → 回退旧页 |
| 掉电在 Step 6（committed 写）完成前 | committed=0 → 回退旧页 |
| 两页均损坏 | 返回 ERR_CORRUPT，使用编译期默认参数 |
| 旧固件 FOC1 格式页 + 新固件启动 | 自动迁移，generation=0，正常工作 |

---

## 相关文件索引

| 文件 | 作用 |
|------|------|
| `Src/HAL/bsp/bsp_flash.h/.c` | Flash 底层 BSP（WriteDoubleWord, CRC32）|
| `Src/UI/parameter/param_storage.h/.c` | 参数存储层（F2 主要修改点）|
| `Src/COMM/protocol/inovxio/inovxio_protocol.h/.c` | CMD 0 广播响应（F1 验证点）|
| `Src/APP/device_id.h` | STM32 96-bit UID（F1 节点唯一标识）|
| `Src/COMM/protocol/inovxio/PROTOCOL_CN.md` | 协议文档（参考 CMD 0 格式）|
