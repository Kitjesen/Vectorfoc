/**
 * @file foc_algorithm.h
 * @brief FOC (磁场定向控制) 纯算法接口
 *
 * @version 1.0
 * @date 2026-01-21
 * @author VectorFOC Team
 *
 * @note 设计原则:
 *   - 纯函数设计，无副作用
 *   - 不访问全局变量
 *   - 无硬件依赖
 *   - 完全可测试
 *   - 平台无关
 */

#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 数据结构定义
 * ============================================================================
 */

typedef struct {
  /* === 传感器反馈 === */
  float Ia;         ///< A相电流 [A]
  float Ib;         ///< B相电流 [A]
  float Ic;         ///< C相电流 [A]
  float Vbus;       ///< 母线电压 [V]
  float theta_elec; ///< 电角度 [rad], 范围 [-PI, PI]
  float omega_elec; ///< 电角速度 [rad/s]

  /* === 控制参考值 === */
  float Id_ref; ///< D轴电流参考值 [A]
  float Iq_ref; ///< Q轴电流参考值 [A]

  /* === 使能标志 === */
  bool enabled; ///< 算法使能标志
} FOC_AlgorithmInput_t;

/**
 * @brief FOC算法输出数据
 *
 * 包含FOC算法计算的所有输出结果。
 * 应用到硬件是调用者的责任（PWM更新等）。
 */
typedef struct {
  /* === PWM占空比输出 === */
  float Ta; ///< A相占空比 [0~1]
  float Tb; ///< B相占空比 [0~1]
  float Tc; ///< C相占空比 [0~1]

  /* === 中间变量（用于调试/监控） === */
  float Ialpha; ///< Alpha轴电流 [A]
  float Ibeta;  ///< Beta轴电流 [A]
  float Id;     ///< D轴电流 [A]
  float Iq;     ///< Q轴电流 [A]
  float Vd;     ///< D轴电压 [V]
  float Vq;     ///< Q轴电压 [V]
  float Valpha; ///< Alpha轴电压 [V]
  float Vbeta;  ///< Beta轴电压 [V]

  /* === 状态标志 === */
  bool overmodulation;    ///< 过调制标志
  bool voltage_saturated; ///< 电压饱和标志
  bool current_limited;   ///< 电流限制标志

} FOC_AlgorithmOutput_t;

/**
 * @brief FOC算法配置参数
 *
 * 运行时不可变的配置参数。
 * 在初始化期间设置并在运行期间保持不变。
 */
typedef struct {
  /* === 电机参数 === */
  float Rs;           ///< 相电阻 [Ohm]
  float Ls;           ///< 相电感 [H]
  float flux;         ///< 磁链 [Wb]
  uint8_t pole_pairs; ///< 极对数

  /* === D轴电流环PI参数 === */
  float Kp_current_d; ///< D轴比例增益
  float Ki_current_d; ///< D轴积分增益

  /* === Q轴电流环PI参数 === */
  float Kp_current_q; ///< Q轴比例增益
  float Ki_current_q; ///< Q轴积分增益

  /* === 控制周期 === */
  float Ts_current; ///< 电流环采样周期 [s]

  /* === 限制参数 === */
  float voltage_limit; ///< 电压限制 [V]
  float current_limit; ///< 电流限制 [A]

  /* === 可选功能开关 === */
  bool enable_decoupling; ///< 使能DQ解耦前馈
  float decoupling_gain;  ///< 解耦增益 (0.0 到 1.0)
  /* === 电流环参数 === */
  float Kb_current;        ///< 抗积分饱和增益 [1/s] (例如 Ki/Kp)
  float current_filter_fc; ///< 电流滤波器截止频率 [Hz]

} FOC_AlgorithmConfig_t;

typedef struct {
  /* === PI控制器积分项 === */
  float integral_d; ///< D轴电流环积分项
  float integral_q; ///< Q轴电流环积分项

  /* === 滤波器状态（可选） === */
  float Id_filt; ///< 滤波后的D轴电流 [A]
  float Iq_filt; ///< 滤波后的Q轴电流 [A]

} FOC_AlgorithmState_t;

/* ============================================================================
 * 核心算法函数
 * ============================================================================
 */

void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state);

/**
 * @brief FOC核心算法计算
 * 计算流程:
 *   1. Clarke变换: abc -> alpha-beta
 *   2. Park变换: alpha-beta -> dq
 *   3. PI电流控制: (Id_ref, Iq_ref) -> (Vd, Vq)
 *   4. 前馈解耦 (可选)
 *   5. 电压限制和抗积分饱和
 *   6. 逆Park变换: dq -> alpha-beta
 *   7. SVPWM调制: alpha-beta -> abc PWM
 *
 * @param input    输入数据（传感器反馈 + 控制参考）
 * @param config   算法配置（不可变）
 * @param state    算法内部状态（可变）
 * @param output   输出数据（计算结果）
 */
void FOC_Algorithm_CurrentLoop(const FOC_AlgorithmInput_t *input,
                               const FOC_AlgorithmConfig_t *config,
                               FOC_AlgorithmState_t *state,
                               FOC_AlgorithmOutput_t *output);

/**
 * @brief 重置FOC算法状态
 *
 * 清除积分项和滤波器状态。
 * 通常在以下情况调用：
 * - 切换使能状态
 * - 切换模式
 * - 检测到故障
 *
 * @param state 算法状态指针
 */
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state);

/**
 * @brief 验证配置参数
 *
 * 检查配置参数是否在合理范围内。
 * 验证包括：
 * - 电机参数 (Rs, Ls, flux, pole_pairs)
 * - PI增益 (Kp, Ki)
 * - 采样周期 (Ts)
 * - 限制 (voltage_limit, current_limit)
 *
 * @param config 配置指针
 * @return true: 有效, false: 无效
 */
bool FOC_Algorithm_ValidateConfig(const FOC_AlgorithmConfig_t *config);

/* ============================================================================
 * 辅助函数（可选）
 * ============================================================================
 */

/**
 * @brief 计算DQ电流环PI增益
 *
 * 根据电机参数和带宽自动计算PI增益。
 * 使用内模控制 (IMC) 方法。
 *
 * @param Rs        相电阻 [Ohm]
 * @param Ls        相电感 [H]
 * @param bandwidth 期望带宽 [Hz]
 * @param Kp        输出比例增益指针
 * @param Ki        输出积分增益指针
 */
void FOC_Algorithm_CalculateCurrentGains(float Rs, float Ls, float bandwidth,
                                         float *Kp, float *Ki);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ALGORITHM_H */
