# Parameter Module (参数管理模块)

**Status**: Active — host-tested; hardware Flash fault-injection pending

## 概述

Parameter 模块集中管理 VectorFOC 固件的运行参数。它负责：

- 参数表初始化、默认值加载和元数据查询；
- 类型安全读写、范围校验、读写权限校验；
- 将通信链路上的 `float` 表示转换为参数表中的实际类型；
- 双页事务化 Flash 持久化和旧版数据迁移。

参数写入后的运行时生效由 APP 的 `RuntimeSettings` adapter 负责。参数
模块只在成功写入 RAM 后发布参数索引；它不直接控制 PID、协议、CAN timeout
或编码器 offset HAL。

应用层优先使用 `param_access.h` 中的接口。`param_table.h` 和 `param_storage.h` 主要供参数模块内部、测试代码或低层维护逻辑使用。

## 模块分层

| 层 | 文件 | 职责 |
| --- | --- | --- |
| Access | `param_access.h/.c` | 公开读写 interface、范围校验、保存/恢复、运行时变更通知 |
| Table | `param_table.h/.c` | 参数索引、类型、默认值、上下限、实际变量指针 |
| Storage | `param_storage.h/.c` | 双页 Flash 镜像、CRC、generation、commit 标记、旧格式迁移 |
| Runtime adapter | `Src/APP/settings/runtime_settings.h/.c` | 在 motor、encoder、protocol 就绪后，把变更应用到运行时对象 |

## 初始化流程

应用启动时调用一次：

```c
ParamResult result = Param_SystemInitOnce();
if (result != PARAM_OK) {
    /* Flash 中没有有效参数或恢复失败时，系统继续使用表中默认值。 */
}
```

`Param_SystemInitOnce()` 会先执行 `ParamTable_Init()`，再尝试 `Param_LoadFromFlash()`。该函数具备一次性保护，重复调用会直接返回 `PARAM_OK`。启动阶段它只恢复参数值；APP 会在 encoder、protocol 和 motor runtime 都就绪后安装 `RuntimeSettings` adapter，并调用 `Param_ApplyRuntimeState()` 统一重放。

如果需要恢复出厂值：

```c
ParamResult result = Param_RestoreDefaults();
if (result == PARAM_OK) {
    (void)Param_SaveToFlash();
}
```

`Param_RestoreDefaults()` 会先校验所有可写参数的默认值，再批量写入内存参数，并在批量结束后统一应用运行时状态。

## 公开访问 API

### 强类型读写

当调用方知道参数的实际类型时，直接使用强类型接口：

```c
float current_limit = 0.0f;
ParamResult read_result = Param_ReadFloat(PARAM_LIMIT_CURRENT, &current_limit);

ParamResult write_result = Param_WriteFloat(PARAM_LIMIT_CURRENT, 20.0f);
```

当前公开的强类型接口包括：

```c
ParamResult Param_ReadFloat(uint16_t index, float *value);
ParamResult Param_WriteFloat(uint16_t index, float value);

ParamResult Param_ReadUint8(uint16_t index, uint8_t *value);
ParamResult Param_WriteUint8(uint16_t index, uint8_t value);

ParamResult Param_ReadUint16(uint16_t index, uint16_t *value);
ParamResult Param_WriteUint16(uint16_t index, uint16_t value);

ParamResult Param_ReadUint32(uint16_t index, uint32_t *value);
ParamResult Param_WriteUint32(uint16_t index, uint32_t value);

ParamResult Param_ReadInt32(uint16_t index, int32_t *value);
ParamResult Param_WriteInt32(uint16_t index, int32_t value);
```

### Runtime adapter seam

`Param_SetRuntimeApplyCallback()` 为 APP 提供一个小 interface。成功的单次
写入通知对应参数索引；从 Flash 批量恢复、恢复默认值或 APP 显式重放时，只
通知一次 `PARAM_RUNTIME_APPLY_ALL`。callback 在参数 critical section 之外
执行，且不得递归写参数。

这些接口会检查：

- 参数索引是否存在；
- 目标类型是否与参数表声明一致；
- 指针是否为空；
- 写入权限；
- 写入值是否有限且落在 `[min, max]` 内。

### 通信链路 float 转换 API

VOFA 和 CAN executor 等通信链路使用 `float` 作为线协议表示。它们应调用：

```c
float wire_value = 0.0f;
ParamResult read_result = Param_ReadAsFloat(PARAM_CAN_TIMEOUT, &wire_value);

ParamResult write_result = Param_WriteFromFloat(PARAM_CAN_TIMEOUT, 1000.0f);
```

`Param_WriteFromFloat()` 会根据参数表中的 `ParamType` 转换实际类型。整数目标必须满足：

- 输入为有限值；
- 输入为整数值，不能有小数部分；
- 输入落在目标整数类型范围和参数表范围内。

因此 `7.5f` 写入 `PARAM_CAN_ID` 会返回 `PARAM_ERR_OUT_OF_RANGE`，不会截断为 `7`。

### Raw 读写边界

`Param_ReadRaw()` 和 `Param_WriteRaw()` 当前是 `param_access.c` 内部 `static` 函数，不是公开 API。外部代码不要声明或调用它们；新增调用点应使用强类型接口或 `Param_ReadAsFloat()` / `Param_WriteFromFloat()`。

## 持久化 API

```c
ParamResult Param_SaveToFlash(void);
ParamResult Param_LoadFromFlash(void);
void Param_ScheduleSave(void);
bool Param_ProcessScheduledSave(void);
ParamResult Param_RollbackScheduledSave(void);
```

- `Param_SaveToFlash()` 从当前参数表收集 `FlashParamData`，先进行完整校验，再调用 `ParamStorage_Save()`。
- `Param_LoadFromFlash()` 从存储层读取有效镜像，完整校验后一次性恢复到运行时参数。
- `Param_ScheduleSave()` 只设置待保存标志，可用于不适合立即擦写 Flash 的路径。
- `Param_ProcessScheduledSave()` 在任务上下文中处理挂起保存；保存失败时会重新置位，后续继续重试。
- 命令服务对同一保存代际最多重试三次；终态失败时会丢弃该代际，再调用 `Param_RollbackScheduledSave()`。
- `Param_RollbackScheduledSave()` 重新加载当前双页存储中最新的有效提交镜像；若没有可加载镜像，则恢复默认参数并清除编码器校准有效状态。它不是完整的 RAM 快照回滚。直接标定路径调用的 `Param_ScheduleSave()` 也使用这条终态回滚路径。

存储层当前签名为：

```c
FlashStorageResult ParamStorage_Save(FlashParamData *data);
FlashStorageResult ParamStorage_Load(FlashParamData *data);
```

注意：`ParamStorage_Save()` 会写入镜像头字段（magic、version、generation、committed、crc32），因此参数是 `FlashParamData *`，不是 `const FlashParamData *`。

## 事务化 Flash 存储

`param_storage.c` 使用两个物理 Flash 页：

- `FLASH_PARAM_PAGE1_ADDR`：Page 62；
- `FLASH_PARAM_PAGE2_ADDR`：Page 63。

保存流程：

1. 选择当前有效页，计算下一代 `generation`；
2. 写入备用页；
3. 写入除 commit doubleword 外的镜像内容；
4. 校验 pending 镜像；
5. 最后写入 commit doubleword；
6. 重新检查新页，确认 CRC、commit 标记和 generation 有效。

加载流程会检查两个页，选择 generation 更新的有效页。有效性条件包括 magic、版本、commit 标记、CRC，以及镜像尾部填零状态。

存储层还能迁移旧版 `FLASH_MAGIC_WORD` 镜像：加载旧格式后会补齐 CAN 波特率、LADRC、编码器校准等新增字段，并归一化为当前 `FLASH_MAGIC_WORD_V2` 镜像。

## 运行时副作用

参数写入成功后，`param_access.c` 会根据参数索引同步运行时状态。当前副作用包括：

| 参数 | 副作用 |
| --- | --- |
| `PARAM_MOTOR_RS`、`PARAM_MOTOR_LS`、`PARAM_MOTOR_FLUX`、`PARAM_MOTOR_POLE_PAIRS` | 标记 `motor_data.params_updated` |
| `PARAM_CUR_KP`、`PARAM_CUR_KI`、`PARAM_LIMIT_CURRENT`、`PARAM_LIMIT_SPEED` | 重新应用电流环配置 |
| `PARAM_SPD_KP`、`PARAM_SPD_KI` | 清空速度 PID 状态 |
| `PARAM_POS_KP` | 清空位置 PID 状态 |
| `PARAM_ADD_OFFSET` | 调用编码器 offset 应用逻辑 |
| `PARAM_CAN_TIMEOUT` | 更新 CAN timeout 检测阈值 |
| `PARAM_LADRC_ENABLE`、`PARAM_LADRC_OMEGA_O`、`PARAM_LADRC_OMEGA_C`、`PARAM_LADRC_B0`、`PARAM_LADRC_MAX_OUT` | 重新初始化 LADRC 配置 |
| `PARAM_RUN_MODE` | 映射并更新 `motor_data.state.Control_Mode` |
| `PARAM_PROTOCOL_TYPE` | 调用 `Protocol_SetType()` |

Flash 加载和恢复默认值会延迟单项副作用，在批量写入结束后统一调用运行时状态应用逻辑，避免中间态被控制环读取。

## 错误码

| 枚举 | 含义 |
| --- | --- |
| `PARAM_OK` | 成功 |
| `PARAM_ERR_INVALID_INDEX` | 参数索引不存在 |
| `PARAM_ERR_INVALID_TYPE` | 参数类型不匹配或不支持 |
| `PARAM_ERR_READONLY` | 试图写入只读参数 |
| `PARAM_ERR_OUT_OF_RANGE` | 写入值非法、非有限值、整数转换不精确或超出范围 |
| `PARAM_ERR_NULL_PTR` | 传入空指针 |
| `PARAM_ERR_STORAGE` | 非易失性存储读写、擦除、校验或镜像选择失败 |

## 元数据查询

通信协议或调试 UI 需要参数名称、范围、默认值时使用：

```c
const ParamEntry *entry = NULL;
if (Param_GetInfo(PARAM_LIMIT_CURRENT, &entry) == PARAM_OK) {
    /* entry->name, entry->min, entry->max, entry->default_val */
}
```

遍历参数表时使用 `ParamTable_GetTable()` 和 `ParamTable_GetCount()`。普通应用逻辑不要直接改写 `ParamEntry::ptr` 指向的数据，应通过 Access 层写入，以保证校验和副作用生效。

## 已验证测试

参数模块当前由主机侧测试覆盖关键行为：

- `test_runner_param_typed_access`：强类型访问、float/整数转换、非有限值拒绝、保存重试、运行时副作用、已提交镜像回滚和默认值回退；
- `test_runner_param_storage`：双页事务写入、CRC、commit、generation、旧格式迁移；
- `test_runner_cmd_service_persistent_rollback`：命令保存与直接标定式保存的三次失败终态、代际清理和运行时回滚；
- `test_runner_comm_executor` 和 `test_runner_vofa_commands`：通信链路通过 `Param_ReadAsFloat()` / `Param_WriteFromFloat()` 访问参数。
