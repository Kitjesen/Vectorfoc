# VectorFOC 项目优化方向研究报告

> **文档版本**: v1.0  
> **日期**: 2026-02-08  
> **基于**: 项目代码审查 + 2024-2026 年学术论文检索

---

## 一、项目现状评估

### 已实现特性

| 特性 | 当前实现 | 评级 | 备注 |
|------|---------|:----:|------|
| FOC 核心算法 | SVPWM（中点注入零序） | ★★★★★ | DC 利用率优秀，5% 安全裕量 |
| 电流环 | PI 控制，20kHz | ★★★★★ | 带宽自整定（Kp=Ls·bw, Ki=Rs·bw） |
| 外环（速度/位置） | PI/P 控制，5kHz | ★★★★☆ | 分频比硬编码为 4 |
| DQ 轴解耦 | 交叉耦合前馈补偿 | ★★★★☆ | 增益可配置 (0~1.0) |
| 齿槽转矩补偿 | 360 点查找表 | ★★★★★ | 完整校准流程 |
| 前馈控制 | 惯量 + 粘滞摩擦 | ★★★☆☆ | 缺少库仑摩擦项 |
| 死区补偿 | 电流方向符号判断 | ★★☆☆☆ | 低电流过零区域精度差 |
| 弱磁控制 | 简单线性斜坡 | ★★☆☆☆ | 无电压环反馈，无 MTPV |
| 滑模观测器 (SMO) | 已实现，编码器为主 | ★★★☆☆ | 存在但未深度集成 |
| MTPA | **未实现** | — | Id_ref = 0（仅适用 SPM） |
| 自适应控制 | **未实现** | — | 固定 PI 增益 |
| 在线参数辨识 | **未实现** | — | 仅校准时测量一次 |

### 核心控制参数

```
电流环频率:    20,000 Hz (50μs)
外环频率:      5,000 Hz (200μs，硬编码 4 分频)
高级控制频率:   5,000 Hz
状态机频率:    1,000 Hz
PWM 频率:     20,000 Hz
默认电流环带宽: 1,000 Hz
MCU 主频:     168 MHz (Cortex-M4F, 含 FPU)
```

---

## 二、高优先级优化方向

### 2.1 自抗扰控制 (ADRC) 替代速度环 PI

#### 研究背景

传统 PI 控制器在参数整定后对特定工况有效，但面对负载突变、摩擦非线性、参数漂移等扰动时鲁棒性不足。自抗扰控制（Active Disturbance Rejection Control, ADRC）通过扩展状态观测器（ESO）实时估计"总扰动"并前馈补偿，从根本上解决此问题。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Improved ADRC with Cascade ESO based on QGI for PMSM Current Disturbances Attenuation* | IEEE TIE, 2025 | 级联 ESO 同时抑制周期性与非周期性扰动 |
| *ADRC Controller Based on Improved ESO for Rapid Dynamic Response of PMSM* | IEEE, 2025 | 改进 ESO 提升动态响应速度 |
| *Adaptive Super-Twisting Controller-Based Modified ESO for PMSM* | MDPI Actuators, 2025 | 自适应超螺旋 + 修正 ESO |
| *PMSM ADRC Based on Ultra-Local Modeling and Improved ESO* | Springer, 2025 | 超局部模型 + 改进 ESO + 滑模 |

#### 技术方案

**推荐**: 线性 ADRC (LADRC) 替换速度环 PI

```
传统 PI 速度环:
  速度误差 → PI → Iq_ref

LADRC 速度环:
  速度误差 → PD 控制律 → Iq_ref
      ↑
  ESO 扰动估计 → 前馈补偿（抵消负载扰动、摩擦、参数变化）
```

LADRC 核心结构：

```c
// 线性扩展状态观测器 (LESO) - 二阶系统
// 状态: z1 = 速度估计, z2 = 总扰动估计
// 输入: y = 实际速度, u = Iq 输出

// 观测器更新 (每个控制周期)
float e_obs = z1 - velocity_measured;
float z1_dot = z2 - 2 * wo * e_obs + b0 * Iq_output;
float z2_dot = -wo * wo * e_obs;
z1 += z1_dot * dt;
z2 += z2_dot * dt;

// 控制律 (PD + 扰动补偿)
float e_ctrl = velocity_ref - z1;
float Iq_ref = (kp * e_ctrl - z2) / b0;
```

其中 `wo` 为观测器带宽，`b0` 为控制增益，`kp` 为比例增益。整个算法仅需约 10 次浮点乘法，计算量与 PI 相当。

#### 实施要点

- **b0 参数**: `b0 ≈ 1 / J`（J 为转动惯量），可粗略估计
- **观测器带宽 wo**: 通常取控制带宽的 3~5 倍
- **优势**: 仅需调 2 个参数 (wo, kp)，比 PI 的 Kp/Ki 更直观
- **降级方案**: 可保留 PI 作为备选，通过参数切换

#### 预期收益

- 负载突变恢复时间减少 **40-60%**
- 对电机参数变化不敏感，**鲁棒性显著提升**
- 低速爬行性能改善（摩擦扰动被实时补偿）

---

### 2.2 改进死区补偿

#### 研究背景

当前项目的死区补偿仅基于电流方向符号，在电流过零点附近会产生严重的电压畸变（第 5、7 次谐波），导致低速时转矩脉动和电流波形畸变。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Improved Dead Time Compensation Based on Multiple Cascaded ESO for PMSM* | IEEE JESTPE, 2025 | 级联 ESO 估计死区扰动电压 |
| *Dead-time compensation based on physical model with online self-commissioning via EKF* | Fraunhofer, 2024 | 物理模型 + EKF 在线辨识死区参数 |
| *Open-Loop Dead-Time Compensation for High-Power IPMSM Sensorless Control* | IEEE, 2025 | 开环补偿，适用无感控制 |

#### 技术方案

**方案 A: 基于电流幅值的分段补偿（快速改进）**

```c
// 替代简单的 sign() 补偿
// 在电流过零区域使用线性过渡，避免阶跃跳变
float deadtime_comp(float current, float dt_voltage) {
    const float I_THRESHOLD = 0.5f;  // 过渡区阈值 (A)

    if (current > I_THRESHOLD)
        return dt_voltage;
    else if (current < -I_THRESHOLD)
        return -dt_voltage;
    else
        // 线性过渡区，避免过零跳变
        return dt_voltage * (current / I_THRESHOLD);
}
```

**方案 B: 基于 ESO 的在线死区估计（推荐方案）**

```
电流环输入 → FOC 算法 → 理论电压 Vdq_ref
                                ↓
                          SVPWM + 逆变器（含死区）
                                ↓
实际电流 Idq ← Clark/Park ← ADC 采样
       ↓
ESO 估计扰动电压 Vdq_disturbance (包含死区效应)
       ↓
Vdq_ref_compensated = Vdq_ref - Vdq_disturbance
```

#### 预期收益

- 低速电流 THD 降低 **40-60%**
- 低速转矩脉动降低 **30-50%**
- 无感控制低速性能显著改善

---

### 2.3 谐波电流注入抑制转矩脉动

#### 研究背景

项目已有的齿槽转矩补偿（360 点查找表）是**位置域**补偿，主要消除齿槽效应。但电磁转矩脉动（由反电动势非正弦性引起的 6 次、12 次谐波）需要**频率域**补偿。两者叠加可实现更极致的平滑性。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Torque Ripple Modeling and Mitigation by Harmonic Current Injection Based on Airgap Field Modulation* | IEEE, 2025 | 气隙场调制理论建模 + 谐波注入 |
| *Flux-Based Harmonic Current Injection Considering Magnetic Saturation* | EPE, 2025 | **12 次转矩脉动降低 >70%** |
| *Torque Ripple Reduction with Novel Harmonic Current Control* | IEEE TIE, 2021 | 新型谐波电流控制器设计 |

#### 技术方案

```c
// 在电流环参考值上叠加 6 次谐波补偿
// theta_e: 电角度, omega_e: 电角频率
float theta_6 = 6.0f * theta_e;

// 补偿系数通过离线标定或在线辨识获得
float Iq_6th_comp = I6_amp * cosf(theta_6 + I6_phase);
float Id_6th_comp = I6_amp_d * sinf(theta_6 + I6_phase_d);

// 叠加到参考电流
Iq_ref_total = Iq_ref + Iq_6th_comp;  // 频率域补偿
Iq_ref_total += cogging_comp_table[pos_index];  // 位置域补偿（已有）
```

**补偿系数标定方法**:
1. 以恒定速度运行电机，记录 Iq 波动
2. 对 Iq 波动做 FFT，提取 6 次和 12 次分量的幅值和相位
3. 将反相分量作为补偿系数写入参数表

#### 预期收益

- 与已有齿槽补偿互补，总转矩脉动降低 **60-80%**
- 低速运行更加丝滑
- 对机器人关节力控应用价值极大

---

## 三、中优先级优化方向

### 3.1 MTPA + 改进弱磁控制

#### 研究背景

当前 Id_ref = 0 的控制策略仅对表贴式永磁电机（SPM）最优。对于内置式永磁电机（IPM，Ld ≠ Lq），利用磁阻转矩可在相同电流下获得更大转矩。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *A Robust Unified Strategy for MTPA and FW in PMSM* | IEEE, 2024 | MTPA-弱磁统一框架，避免切换不连续 |
| *Novel MTPA-FW Feedforward Strategy Based on Online Model Parameter Update* | SAE, 2022 | 在线参数更新，瞬态响应快 60% |
| *Extended MTPA-FW Control for PM Machines with CSI* | IEEE, 2024 | 扩展到电流源逆变器 |

#### 技术方案

**MTPA 查找表法**（适合 STM32G4 实时性要求）:

```c
// 预计算 MTPA 查找表: Iq → 最优 Id
// 对于 IPM 电机: Id_mtpa = (flux - sqrt(flux^2 + 8*(Lq-Ld)^2 * Iq^2)) / (4*(Lq-Ld))
static float mtpa_id_table[MTPA_TABLE_SIZE];

// 运行时直接查表 + 线性插值
float Id_ref = MTPA_LookupId(Iq_ref);
```

**统一弱磁控制**（替代当前线性斜坡）:

```
                 ┌─────────────┐
 Iq_ref ──→     │  电压圆限制  │ ──→ Id_ref (自动弱磁)
 Vbus   ──→     │  电流圆限制  │ ──→ Iq_ref_limited
 omega_e ──→    │  MTPA 约束   │
                 └─────────────┘
```

#### 预期收益

- IPM 电机效率提升 **10-20%**（相同转矩下电流更小）
- 弱磁区间扩展，高速性能改善
- MTPA-弱磁平滑过渡，无切换冲击

---

### 3.2 在线参数辨识

#### 研究背景

电机参数（Rs, Ls, flux）随温度和工况变化。当前仅在校准时测量一次，长时间运行后参数漂移会导致电流环性能下降、观测器精度降低。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Sensorless IPMSM Drive with Parameter Identification using Current Prediction Error Gradients* | EPE, 2025 | 电流预测误差梯度法辨识 Rs 和 flux |
| *Online Multiple Parameter Identification with Phase Voltage Measurements* | IEEE TIE, 2020 | 多参数同时在线辨识 |
| *Combined Deep Learning and RLS for Real-Time Inductance Estimation* | MCA, 2025 | DL + RLS 混合辨识 |

#### 推荐方案: 递推最小二乘法 (RLS)

```c
// 基于 DQ 模型的在线参数辨识
// Vd = Rs*Id + Ls*dId/dt - omega_e*Ls*Iq
// Vq = Rs*Iq + Ls*dIq/dt + omega_e*(Ls*Id + flux)

// RLS 辨识 [Rs, Ls, flux] 三个参数
// 回归向量: phi = [Id, dId/dt - omega*Iq; Iq, dIq/dt + omega*Id; omega]
// 输出: y = [Vd; Vq]

typedef struct {
    float theta[3];     // 参数估计 [Rs, Ls, flux]
    float P[3][3];      // 协方差矩阵
    float lambda;       // 遗忘因子 (0.995 ~ 0.999)
} RLS_Identifier_t;
```

#### 辨识结果应用

```
Rs 更新 → 电流环 Ki 自动调整 (Ki = Rs * bandwidth)
Ls 更新 → 电流环 Kp 自动调整 (Kp = Ls * bandwidth)
         → DQ 解耦前馈更新
flux 更新 → 力矩估计精度提升
          → 弱磁控制阈值更新
```

#### 预期收益

- 全温度范围 (-20°C ~ 80°C) 控制性能一致
- 电流环带宽保持在最优值
- 为 MTPA 和弱磁控制提供实时参数支持

---

### 3.3 自适应滑模观测器改进

#### 研究背景

项目已有 SMO 实现，但传统 SMO 的 `sign()` 切换函数在低速时产生严重抖振，限制了其实用性。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Adaptive Sliding Mode Observers for Speed Sensorless Control* | MDPI Energies, 2025 | 模糊自适应 + 曲线拟合自适应增益 |
| *Dynamic Self-Adjusting System Using Improved Super-Twisting SMO* | MDPI Sensors, 2025 | 改进超螺旋算法，收敛更快 |
| *Improved Extraction Scheme for HF Injection in Sensorless PMSM* | MDPI EVS, 2025 | EMA 滤波改善 HFI 提取 |
| *Improved Rotor Flux Observer for Dual Three-Phase PMSM* | IET EPA, 2025 | 高通/低通混合磁链观测器 |

#### 技术方案: 混合无感控制

```
速度区域划分:
  ┌───────────────────────────────────────────────────┐
  │  [0 ~ 5% 额定]    HFI 高频注入 (方波/正弦波)     │
  │  [5% ~ 15% 额定]  HFI + SMO 加权过渡             │
  │  [15% ~ 100%]     改进 SMO (自适应增益)           │
  └───────────────────────────────────────────────────┘
```

**改进 SMO 核心**: 用 `sigmoid()` 替代 `sign()` 减少抖振:

```c
// 传统 SMO (抖振严重)
float sw = (e > 0) ? k : -k;

// 改进: 自适应 sigmoid 函数
float sigmoid_adaptive(float e, float delta) {
    return e / (fabsf(e) + delta);
    // delta 随速度自适应: 低速大 (平滑), 高速小 (快响应)
}
```

#### 预期收益

- 实现全速域无感控制（包括零速启动）
- 作为编码器故障的降级备用方案
- 降低系统成本（可选去掉编码器）

---

## 四、长期探索方向

### 4.1 模型预测电流控制 (MPC)

#### 研究背景

MPC 利用电机模型预测下一拍电流，直接计算最优电压矢量，理论上可实现比 PI 更快的动态响应和更低的谐波。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Model-Free Predictive Current Control Using Extended Affine Ultralocal* | IEEE TIE, 2024 | 无需电机模型的预测控制 |
| *Over-Modulated Model Predictive Current Control for PMSM* | IEEE, 2022 | 过调制区 MPC |
| *Implementation of CCS-MPC on FPGA* | IEEE, 2023 | FPGA 实时实现 |

#### 可行性评估

```
STM32G4 @ 168MHz 算力分析:
  - 20kHz 控制周期 = 50μs 可用
  - 当前 FOC + PI 占用约 ~15μs
  - 标准 MPC (8 矢量遍历) 约需 ~25-30μs
  - 结论: 勉强可行，但裕量小

替代方案:
  - 无模型 MPC (Model-Free MPC): 计算量减少 40%
  - 降频 MPC: 10kHz MPC + 20kHz PWM
  - 简化 MPC: 仅 3 矢量候选
```

**建议**: 先做仿真验证，评估收益是否值得算力投入。

---

### 4.2 自适应陷波滤波器（机械谐振抑制）

#### 研究背景

机器人关节驱动系统中，减速器弹性、联轴器间隙等会引入机械谐振，导致速度环/位置环振荡。

#### 关键论文

| 论文 | 来源 | 核心贡献 |
|------|------|---------|
| *Adaptive Vibration Suppression for Direct-Drive Flexible Manipulators* | IEEE, 2025 | PMSM 自感知振动抑制 |
| *Suppress Resonances Using ESC and FRE Based ANF* | MathWorks, 2025 | 自适应陷波 + 频响估计 |
| *Discretization with Phase Compensation for ANF-Based Resonance Suppression* | IEEE, 2020 | 离散化相位补偿 |

#### 技术方案

```c
// 二阶陷波滤波器 (Notch Filter)
// H(z) = (z^2 - 2*cos(wn*T)*z + 1) / (z^2 - 2*r*cos(wn*T)*z + r^2)
// wn: 陷波频率, r: 陷波深度 (0.9~0.99)
typedef struct {
    float wn;       // 谐振频率 (rad/s)
    float r;        // 极点半径 (控制陷波宽度)
    float x[2];     // 输入历史
    float y[2];     // 输出历史
} NotchFilter_t;

// 自适应: 通过 FFT 或自相关分析在线检测谐振频率
// 当检测到谐振时自动调整 wn
```

#### 适用场景

- 带减速器的关节驱动
- 长轴传动系统
- 柔性负载应用

---

### 4.3 前馈控制增强

#### 当前不足

项目前馈仅包含惯量和粘滞摩擦，缺少：
- **库仑摩擦**: `T_coulomb = sign(vel) * Fc`
- **重力补偿**: `T_gravity = m * g * L * sin(theta)`（关节驱动）
- **Stribeck 效应**: 低速摩擦非线性模型

#### 改进方案

```c
// 完整摩擦模型前馈
float friction_feedforward(float velocity, float acceleration) {
    float T_inertia  = J * acceleration;              // 惯量 (已有)
    float T_viscous  = B * velocity;                   // 粘滞 (已有)
    float T_coulomb  = Fc * tanhf(velocity / v_stribeck); // 库仑 + Stribeck
    return T_inertia + T_viscous + T_coulomb;
}
```

使用 `tanh()` 替代 `sign()` 避免速度零点不连续。

---

## 五、实施路线图

```
═══════════════════════════════════════════════════════════════
 Phase 1: 基础改进 (1~2 个月)                    优先级: 高
═══════════════════════════════════════════════════════════════

 [Week 1-2]  改进死区补偿
             ├── 实现分段线性过渡补偿
             ├── 验证低速电流 THD 改善
             └── 对比测试: sign() vs sigmoid()

 [Week 3-4]  外环分频比可配置化
             ├── 将硬编码 4 改为参数表配置
             └── 增加前馈库仑摩擦项

 [Week 5-8]  LADRC 速度环
             ├── 实现 LESO (线性扩展状态观测器)
             ├── 实现 LADRC 控制律
             ├── 参数整定与对比测试
             └── 保留 PI 作为降级选项

═══════════════════════════════════════════════════════════════
 Phase 2: 性能提升 (2~4 个月)                    优先级: 中
═══════════════════════════════════════════════════════════════

 [Month 3]   谐波电流注入
             ├── 搭建转矩脉动 FFT 分析工具
             ├── 实现 6 次谐波前馈补偿
             └── 与齿槽补偿叠加测试

 [Month 3]   MTPA + 弱磁控制重构
             ├── 实现 MTPA 查找表
             ├── 基于电压圆的弱磁控制
             └── 统一 MTPA-FW 框架

 [Month 4]   在线参数辨识 (RLS)
             ├── 实现 Rs/Ls/flux 三参数 RLS
             ├── 辨识结果自动更新控制器增益
             └── 温度变化场景验证

═══════════════════════════════════════════════════════════════
 Phase 3: 高级功能 (4~6 个月)                    优先级: 低
═══════════════════════════════════════════════════════════════

 [Month 5]   改进 SMO + HFI 混合无感控制
             ├── 自适应 sigmoid SMO
             ├── 方波 HFI 低速方案
             └── 加权过渡策略

 [Month 6]   MPC 可行性验证 / 自适应陷波滤波
             ├── 离线仿真 MPC 算力评估
             ├── 简化 MPC 原型实现
             └── 机械谐振检测与 ANF 实现
```

---

## 六、参考论文汇总

### ADRC / ESO 方向

1. *Improved ADRC with a Cascade Extended State Observer Based on Quasi-Generalized Integrator for PMSM Current Disturbances Attenuation* — IEEE, 2025
2. *ADRC Controller Based on Improved ESO for Rapid Dynamic Response of PMSM* — IEEE, 2025
3. *Adaptive Super-Twisting Controller-Based Modified Extended State Observer for Permanent Magnet Synchronous Motors* — MDPI Actuators, 2025
4. *Permanent Magnet Synchronous Motor ADRC Based on Ultra-Local Modeling and Improved Extended State Observer* — Springer, 2025

### 死区补偿方向

5. *Improved Dead Time Compensation Method Based on Multiple Cascaded Extended State Observer for PMSM Drives* — IEEE JESTPE, 2025
6. *Dead-time Compensation Based on a Physical Model with Online Self-Commissioning Procedure Based on an Extended Kalman Filter* — Fraunhofer, 2024
7. *Open-Loop Dead-Time Compensation for High-Power IPMSM Sensorless Control* — IEEE, 2025

### 转矩脉动抑制方向

8. *Torque Ripple Modeling and Mitigation for PMSM by Harmonic Current Injection Based on General Airgap Field Modulation Theory* — IEEE, 2025
9. *Flux-Based Harmonic Current Injection Method Considering Magnetic Saturation* — EPE, 2025 (12 次脉动降低 >70%)
10. *Torque Ripple Reduction Method for PMSM Drives With Novel Harmonic Current Control* — IEEE TIE, 2021

### MTPA / 弱磁方向

11. *A Robust Unified Strategy for Maximum Torque per Ampere and Field Weakening in PMSM* — IEEE, 2024
12. *A Novel MTPA-Flux Weakening Feedforward Control Strategy Based on On-line Model Parameter Update* — SAE, 2022

### 在线参数辨识方向

13. *Sensorless IPMSM Drive with Parameter Identification using Open-Loop Current Prediction Error Gradients* — EPE, 2025
14. *An Embedded Strategy for Online Identification of PMSM Parameters and Sensorless Control* — IEEE, 2018

### 无感控制 / 观测器方向

15. *Adaptive Sliding Mode Observers for Speed Sensorless Induction Motor Control and Their Comparative Performance Tests* — MDPI Energies, 2025
16. *A Dynamic Self-Adjusting System for PMSM Using an Improved Super-Twisting Sliding Mode Observer* — MDPI Sensors, 2025
17. *An Improved Extraction Scheme for High-Frequency Injection in Sensorless PMSM Control* — MDPI EVS, 2025

### 振动抑制方向

18. *Adaptive Vibration Suppression Control for Direct-Drive Flexible Manipulators Based on PMSM Self-Sensing* — IEEE, 2025
19. *Application of Discretization Method with Phase Compensation for Adaptive-Notch-Filter Based Resonance Suppression in Industrial Servo Systems* — IEEE, 2020

### MPC 方向

20. *Model-Free Predictive Current Control Using Extended Affine Ultralocal for PMSM Drives* — IEEE TIE, 2024
21. *An Over-Modulated Model Predictive Current Control for Permanent Magnet Synchronous Motors* — IEEE, 2022

---

## 七、总结

VectorFOC 项目的底层 FOC 实现（SVPWM、电流环自整定、齿槽补偿、DQ 解耦）已达到较高水平。优化方向按 **投入产出比** 排序:

| 排名 | 方向 | 难度 | 收益 | 推荐指数 |
|:----:|------|:----:|:----:|:--------:|
| 1 | 改进死区补偿 | ★★☆ | ★★★★ | ★★★★★ |
| 2 | LADRC 速度环 | ★★★ | ★★★★★ | ★★★★★ |
| 3 | 谐波电流注入 | ★★☆ | ★★★★ | ★★★★☆ |
| 4 | MTPA + 弱磁 | ★★★ | ★★★★ | ★★★★☆ |
| 5 | 在线参数辨识 | ★★★ | ★★★☆ | ★★★☆☆ |
| 6 | 改进 SMO | ★★★★ | ★★★☆ | ★★★☆☆ |
| 7 | MPC 电流控制 | ★★★★★ | ★★★ | ★★☆☆☆ |
| 8 | 自适应陷波 | ★★★ | ★★☆ | ★★☆☆☆ |

> **核心建议**: 先做死区补偿改进和 LADRC，这两项改动范围小、收益大、风险低，是最佳起步点。
