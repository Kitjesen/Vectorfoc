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
- **写入开销受控**：每次保存只擦除 standby 页，写 payload 后追加一次最终 commit doubleword

### 2.3 当前实现：Generation Counter + Erased Commit Doubleword

`FlashParamData` 现在是紧凑 RAM 结构，移除了原先只为填满 Flash 页而存在的 `reserved_data`。兼容性由固定逻辑镜像大小 `PARAM_FLASH_IMAGE_SIZE == FLASH_PARAM_PAGE_SIZE == 2048` 保证；事务字段仍位于 16 字节头之后：

```c
typedef struct {
    uint32_t magic;           /* 0x464F4332 "FOC2" for current format */
    uint32_t param_version;   /* must equal FLASH_PARAM_VERSION */
    uint32_t crc32;           /* CRC32/IEEE over logical bytes [16, 2048) */
    uint32_t reserved;
    uint32_t generation;      /* monotonically increasing write counter */
    uint32_t committed;       /* 0 = committed, 0xFFFFFFFF = pending/erased */
    /* compact payload fields; flash bytes after sizeof(FlashParamData) are zero */
} FlashParamData;
```

CRC semantics are CRC32/IEEE over the logical 2KB image. The CRC starts at offset 16, so it covers `generation`, `committed`, the compact payload bytes, and a zero-filled tail through `PARAM_FLASH_IMAGE_SIZE`; it skips only `magic`, `param_version`, `crc32`, and `reserved`. Load computes this CRC by streaming small chunks from Flash, and Save constructs it from the compact RAM struct plus a zero tail.

#### 2.3.1 页角色动态交替（Ping-Pong）

```
写第 N 次：
  - 当前 Active Page  = Page A（generation = N-1）
  - 当前 Standby Page = Page B

步骤：
  1. Load 当前有效页；若无有效页，则从 generation=0、Page1 active 状态开始
  2. 构造 final image：magic=FOC2、version=FLASH_PARAM_VERSION、generation=N、committed=0
  3. 将 crc32 清零后，按 offset 16 到 `PARAM_FLASH_IMAGE_SIZE` 计算 CRC32/IEEE（紧凑结构后补零），写入 final image 的 crc32
  4. 构造 pending image：复制 final image，但 generation 和 committed 都保持擦除态 0xFFFFFFFF
  5. Erase standby page
  6. 写入除 offset 16 commit doubleword 之外的所有 doubleword，包括紧凑结构后的零 tail
  7. 用小 doubleword/chunk buffer 验证 pending image：prefix、擦除态 commit doubleword、compact suffix 和零 tail；此时 Flash 上的 commit doubleword 仍应为 0xFFFFFFFFFFFFFFFF
  8. 最后只写一次 commit doubleword：低 32 位为 generation=N，高 32 位为 committed=0
  9. 重新按 FOC2 有效页规则验证新页，新页成为 Active
```

该顺序利用 STM32G4 Flash 擦除后为 1、编程只能将 bit 清 0 的特性：提交 doubleword 在最终提交前完全不写，掉电时会保持擦除态，Load 时不会被当作有效页。

#### 2.3.2 掉电恢复逻辑（Load）

```
读 Page62 和 Page63:
  - FOC2 页：magic、version、committed、CRC 全部有效才接受
  - FOC1 页：只接受明确支持的历史版本并迁移到内存中的 FOC2 形态
  - 两页均有效：用 generation_is_newer() 处理 32-bit 回绕，选择较新的页
  - 只有一页有效：使用该页
  - 均无效：返回 FLASH_STORAGE_ERR_CORRUPT，调用方使用默认参数
```

FOC2 有效页条件：

```c
magic == FLASH_MAGIC_WORD_V2
param_version == FLASH_PARAM_VERSION
committed == 0
crc32 == CRC32_IEEE(logical_bytes[16..PARAM_FLASH_IMAGE_SIZE))
```

#### 2.3.3 Commit Doubleword 布局

`generation` 与 `committed` 必须同处 offset 16 的 64-bit doubleword：

```c
offsetof(FlashParamData, generation) == 16
offsetof(FlashParamData, committed)  == 20
```

保存时先跳过该 doubleword，使 Flash 中保持：

```text
0xFFFFFFFFFFFFFFFF  // pending / erased
```

最终提交只执行一次 doubleword program：

```c
uint64_t commit_dw = ((uint64_t)0u << 32) | (uint64_t)next_generation;
BSP_Flash_WriteDoubleWord(write_addr + 16u, commit_dw);
```

因此任何发生在最终提交前的掉电，都只会留下 pending 页；发生在最终提交后的掉电，则该页必须通过 FOC2 严格版本和 CRC 校验后才会被选中。

### 2.4 Save 流程（与当前 `param_storage.c` 对齐）

```c
static FlashStorageResult ParamStorage_Save_v2(FlashParamData *data) {
    if (data == NULL) return FLASH_STORAGE_ERR_LOCKED;

    PageInfo current;
    uint8_t active_page;
    if (select_best_page(&current, &active_page)) {
        s_generation = current.generation;
        s_active_page = active_page;
    } else {
        s_generation = 0;
        s_active_page = 0;
    }

    uint8_t standby = 1u - s_active_page;
    uint32_t write_addr = page_addr_from_index(standby);
    uint32_t next_generation = s_generation + 1u;

    /* Mutate caller image in place; no full-image stack copy. */
    data->magic = FLASH_MAGIC_WORD_V2;
    data->param_version = FLASH_PARAM_VERSION;
    data->generation = next_generation;
    data->committed = 0;
    data->crc32 = 0;
    data->crc32 = param_crc32(data);  // compact bytes + zero tail to 2048

    BSP_Flash_Unlock();
    if (!BSP_Flash_ErasePage(write_addr)) fail_erase;
    if (!flash_write_image_except_commit_dw(write_addr, data)) fail_write;
    if (!verify_pending_image(write_addr, data)) fail_verify;

    uint64_t commit_dw = ((uint64_t)0u << 32) | (uint64_t)next_generation;
    if (!BSP_Flash_WriteDoubleWord(write_addr + 16u, commit_dw)) fail_write;
    BSP_Flash_Lock();

    PageInfo verify;
    if (!inspect_page(write_addr, &verify) || verify.generation != next_generation) {
        return FLASH_STORAGE_ERR_VERIFY;
    }

    s_generation = next_generation;
    s_active_page = standby;
    s_last_crc = verify.crc32;
    s_write_count++;
    return FLASH_STORAGE_OK;
}
```
### 2.5 Load 流程（与当前 `param_storage.c` 对齐）

```c
static FlashStorageResult ParamStorage_Load_v2(FlashParamData *data) {
    if (data == NULL) return FLASH_STORAGE_ERR_LOCKED;

    PageInfo best;
    uint8_t active_page;
    if (!select_best_page(&best, &active_page)) {
        return FLASH_STORAGE_ERR_CORRUPT;
    }

    /* Only the chosen page is loaded into the caller's compact struct. */
    if (!load_page(best.address, data)) {
        return FLASH_STORAGE_ERR_CORRUPT;
    }

    s_generation = data->generation;
    s_active_page = active_page;
    s_last_crc = data->crc32;
    return FLASH_STORAGE_OK;
}
```
### 2.6 向后兼容迁移

`inspect_page()` / `load_page()` 对 `FOC1` 只接受两个明确的历史布局：

| 版本 | 布局 | 迁移规则 |
|------|------|----------|
| `0x00010001` | 16B header，无 transaction doubleword | 先按 offset 16 CRC 校验旧 payload；校验通过后将 payload 前移到当前 24B header 后，丢弃末尾 8B legacy reserved |
| `0x00010000` | transitional 24B header，`generation=0` 且 `committed=0` | 按当前 offset 16 CRC 校验；校验通过后直接规范化为 FOC2 |

迁移只在内存中规范化读取结果：

```c
out->magic = FLASH_MAGIC_WORD_V2;
out->param_version = FLASH_PARAM_VERSION;
out->generation = 0;
out->committed = 0;
out->crc32 = 0;
out->crc32 = param_crc32(out);
```

其他 `FOC1` 版本、FOC2 版本不匹配、CRC 不匹配、或 pending commit doubleword 未提交的页面均视为无效。

### 2.7 Flash 布局（无变化）

```
STM32G431CB 128KB Flash:
0x08000000 - 0x08003FFF  Bootloader (16KB, 8 pages)
0x08004000 - 0x0801EFFF  Application (108KB, 54 pages)
0x0801F000 - 0x0801F7FF  FLASH_PARAM_PAGE1 (2KB, Page 62) ← 角色动态交替
0x0801F800 - 0x0801FFFF  FLASH_PARAM_PAGE2 (2KB, Page 63) ← 角色动态交替
```

> 当前 `FlashParamData` 为紧凑 RAM 结构；Flash 兼容性由 `PARAM_FLASH_IMAGE_SIZE` 固定为 2KB，并要求结构体大小严格小于该镜像大小。紧凑结构后的 Flash tail 必须写成 0，并参与 CRC。

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
| 1. `param_storage.h`：固定 `FlashParamData` 头部、transaction doubleword 和 2KB 上限断言 | `param_storage.h` | 已实现 |
| 2. `param_storage.c`：Save_v2 / Load_v2 / FOC1 迁移与 generation 回绕选择 | `param_storage.c` | 已实现 |
| 3. `bsp_flash.c`：doubleword 对齐/范围保护和 CRC32/IEEE 软件语义 | `bsp_flash.c` | 已实现 |
| 4. 单元测试：pending 页、最终提交、CRC/迁移和损坏回退场景 | `test/test_param_storage.c`, `test/test_bsp_flash.c` | 已覆盖/补充 |
| **状态** | | **实现已落地，持续由测试矩阵回归** |

**当前状态**：F2 固件实现已落地；本文档记录当前行为与后续验证口径。

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
| 写 payload 过程中掉电 | 新页 commit doubleword 仍为擦除态，Load 回退旧页 |
| pending image verify 前数据损坏 | verify 失败，返回写入错误；旧页保持有效 |
| 最终 commit doubleword 写入前掉电 | 新页保持 pending/erased 状态，Load 回退旧页 |
| 最终 commit doubleword 写入后掉电 | 新页需通过 FOC2 版本和 CRC 校验，通过后按 generation 选中 |
| 两页均损坏或均为 pending | 返回 ERR_CORRUPT，使用编译期默认参数 |
| FOC1 `0x00010001` 16B header 页 | 校验旧 payload 后迁移到内存中的 FOC2/generation 0 形态 |
| FOC1 `0x00010000` transitional 24B header 页 | 校验当前布局后迁移到内存中的 FOC2/generation 0 形态 |

---

## 相关文件索引

| 文件 | 作用 |
|------|------|
| `Src/HAL/bsp/bsp_flash.h/.c` | Flash 底层 BSP（WriteDoubleWord, CRC32）|
| `Src/UI/parameter/param_storage.h/.c` | 参数存储层（F2 主要修改点）|
| `Src/COMM/protocol/inovxio/inovxio_protocol.h/.c` | CMD 0 广播响应（F1 验证点）|
| `Src/APP/device_id.h` | STM32 96-bit UID（F1 节点唯一标识）|
| `Src/COMM/protocol/inovxio/PROTOCOL_CN.md` | 协议文档（参考 CMD 0 格式）|
