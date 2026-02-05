# Parameter Module (参数管理模块)

**Status**: ✅ Production Ready

## 概述

Parameter 模块是 FalconFoc 固件的核心组件之一，负责管理电机控制系统的所有配置参数。它提供了一个统一的接口，用于参数的**定义**、**运行时访问**（读/写）、**范围验证**以及**非易失性存储**（Flash 持久化）。

## 核心架构

该模块采用**表驱动**架构，主要由以下三部分组成：

1. **ParamAccess** (`param_access.h`): 面向应用层的顶层 API，提供类型安全的读写接口和事务逻辑（如校验、保存）。应用层**只应该**与此层交互。
2. **ParamTable** (`param_table.h`): 数据定义层，维护参数注册表、元数据（最小值、最大值、默认值）和物理变量指针。
3. **ParamStorage** (`param_storage.h`): 硬件抽象层，负责 Flash 扇区的擦除、写入、CRC 校验和双页备份管理。

---

## API 详解 (Reference)

### 1. 参数访问层 (`param_access.h`)

这是应用程序调用的主要接口。

#### 基础读写

```c
/**
 * @brief 通用参数读取
 * @param index 参数索引 (如 PARAM_MOTOR_RS)
 * @param data [OUT] 接收数据的缓冲区指针
 * @param type [OUT] 返回该参数的数据类型 (可选，传 NULL 忽略)
 * @return 0=成功, 非0=错误码
 */
ParamResult Param_Read(uint16_t index, void *data, ParamType *type);

/**
 * @brief 通用参数写入
 * @param index 参数索引
 * @param data [IN] 包含待写入数据的缓冲区指针
 * @note 此函数会自动检查数据类型匹配、边界限制 (min/max) 和读写权限 (RO/RW)。
 * @return 0=成功, 非0=错误码
 */
ParamResult Param_Write(uint16_t index, const void *data);
```

#### 类型安全便捷接口

推荐在已知参数类型时使用，比通用接口更安全且无需 `void*` 转换。

```c
// Float 类型参数
ParamResult Param_ReadFloat(uint16_t index, float *value);
ParamResult Param_WriteFloat(uint16_t index, float value);

// Uint8 类型参数
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value);
ParamResult Param_WriteUint8(uint16_t index, uint8_t value);
```

#### 持久化与管理

```c
/**
 * @brief 保存所有参数到 Flash
 * @details 遍历参数表，将所有标记为 PARAM_ATTR_PERSISTENT 的参数序列化并写入 Flash。
 *          使用双页备份机制：通常写入 Page1，若失败则写入 Page2。
 * @return 0=成功, 非0=错误码
 */
ParamResult Param_SaveToFlash(void);

/**
 * @brief 从 Flash 加载参数
 * @details 尝试从 Page1 加载，若校验失败则尝试 Page2。
 *          若均无效，则保持默认值或调用 RestoreDefaults。
 * @return 0=成功, 非0=错误码 (无效数据)
 */
ParamResult Param_LoadFromFlash(void);

/**
 * @brief 恢复出厂设置
 * @details 将所有参数重置为 param_table 中定义的 default_val。
 *          注意：此操作仅在内存中生效，调用 SaveToFlash() 后才持久化。
 */
ParamResult Param_RestoreDefaults(void);

/**
 * @brief 获取参数元数据
 * @details 获取参数的属性（如 Min, Max, Default, Name）。常用于通信协议返回参数描述。
 */
ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry);
```

---

### 2. 参数表层 (`param_table.h`)

主要用于内部查找，通常不直接由应用层调用。

```c
void ParamTable_Init(void);  /* @brief 初始化参数表 */

/**
 * @brief 查找参数条目
 * @param index 索引
 * @return 指向 ParamEntry 的指针，若未找到返回 NULL。
 */
const ParamEntry *ParamTable_Find(uint16_t index);

/**
 * @brief 获取整个参数表
 * @details 用于遍历整个表（例如 Flash 存储模块需要遍历保存）。
 */
const ParamEntry *ParamTable_GetTable(void);

/**
 * @brief 获取参数总数
 */
uint32_t ParamTable_GetCount(void);
```

---

### 3. 存储层 (`param_storage.h`)

Flash 驱动抽象。除非你需要直接操作 Flash，否则应使用 `Param_SaveToFlash`。

```c
/**
 * @brief 初始化存储模块
 * @details 检查 Flash 魔术字，验证两个分区的数据完整性。
 */
void ParamStorage_Init(void);

/**
 * @brief 写入参数数据包
 * @param data 准备好的数据结构体
 * @return 写入结果 (OK / EraseErr / WriteErr)
 */
FlashStorageResult ParamStorage_Save(const FlashParamData *data);

/**
 * @brief 读取参数数据包
 * @param data [OUT] 接收结构体
 * @return 加载结果 (OK / CrcErr / MagicErr)
 */
FlashStorageResult ParamStorage_Load(FlashParamData *data);

/**
 * @brief 擦除参数区
 * @details 擦除主页和备份页。
 */
FlashStorageResult ParamStorage_Erase(void);

/**
 * @brief 检查 Flash 是否包含有效数据
 * @return true = 有效且 CRC 通过
 */
bool ParamStorage_HasValidData(void);
```

---

## 错误代码对照表 (ParamResult)

| 错误枚举                    | 值 | 含义                         |
| :-------------------------- | :- | :--------------------------- |
| `PARAM_OK`                | 0  | 操作成功                     |
| `PARAM_ERR_INVALID_INDEX` | 1  | 索引不存在                   |
| `PARAM_ERR_INVALID_TYPE`  | 2  | 读写的数据类型不匹配         |
| `PARAM_ERR_READONLY`      | 3  | 试图写入只读 (RO) 参数       |
| `PARAM_ERR_OUT_OF_RANGE`  | 4  | 写入值超出了 [Min, Max] 范围 |
| `PARAM_ERR_NULL_PTR`      | 5  | 传入的指针为空               |

---

## 最佳实践示例

### 初始化流程

```c
void System_Init(void) {
    // 1. 初始化表 (内存赋默认值)
    ParamTable_Init();
  
    // 2. 尝试从 Flash 加载
    if (Param_LoadFromFlash() != PARAM_OK) {
        // 若 Flash 无效 (如首台烧录)，可选：显式保存一次默认值
        // Param_SaveToFlash(); 
        printf("Using default parameters.\n");
    } else {
        printf("Parameters loaded.\n");
    }
}
```

### 运行时修改参数

```c
void Update_Current_Limit(float new_limit) {
    // 尝试写入，模块会自动检查 new_limit 是否在 [min, max] 内
    ParamResult res = Param_WriteFloat(PARAM_LIMIT_CURRENT, new_limit);
  
    if (res == PARAM_OK) {
        printf("Success\n");
    } else if (res == PARAM_ERR_OUT_OF_RANGE) {
        printf("Error: Value out of range!\n");
    } else {
        printf("Error: %d\n", res);
    }
}
```
