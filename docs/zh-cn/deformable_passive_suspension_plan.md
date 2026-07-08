# 变形底盘二代被动悬挂方案（Plan）

本文档定义一套从零设计的二代被动悬挂方案。

本文档刻意不以当前仓库中的被动悬挂实现为基础，而是从问题约束、可观测性、控制目标、响应要求和工程落地风险重新设计。

## 1. 目标

二代被动悬挂的目标不是“让四个 joint torque 相等”，也不是“让车身姿态永远水平”，而是：

1. 让每个轮子都尽量保持足够的接地支撑余量。
2. 在不平地形、斜着上坡、坡边界、对角扭转地形下，尽量避免单轮轻载或悬空。
3. 在车身变重、局部冲击、轮端打滑、底盘触地嫌疑等复杂工况下保持可解释、可调、可保护。
4. 在“局部轮子快速卸载”的场景下，局部恢复响应不弱于主动悬挂，目标是优于主动悬挂。
5. 保持与现有 `SPIN`、`STEP_DOWN` 等底盘模式解耦。被动悬挂只接管 joint correction，不接管 chassis mode。

## 2. 已知约束

设计必须严格接受以下事实，不能用过度简化假设替代：

1. joint 机构是平行四边形升降结构，joint torque 与实际轮端支撑反力不是线性关系，并且显著依赖 joint 角度、关节速度、车身姿态和机构偏置。
2. 底盘 IMU 反馈的不是地面真实坡度。车辆可能侧着上坡，也可能经过多个坡的边界处，不能把 `pitch/roll` 当作唯一地形描述。
3. 四个轮子四个接触点不一定共面。地形可能包含 `common / pitch / roll / twist` 四类分量，其中 `twist` 是非共面地形的关键。
4. 小球、碎屑、地面局部凸起会引起短时 joint torque 突变或轮速异常，不能把单个信号突变直接当作“悬空”或“触地”。
5. 轮端打滑和轮端轻载相关，但二者并不等价。打滑只能作为接触置信度的一部分，不能直接当作触地判据。
6. 底盘触地后，受力路径会绕开部分轮端与 joint，导致 joint torque 观测失真，因此“底盘触地检测”只能通过多源证据推断，难以靠单一量严格判断。
7. joint 本地控制器是 ADRC。若想提升响应，必须利用 ADRC 结构优势，例如改进参考轨迹、加速度前馈、扭矩前馈、参数调度、事件触发重置等，而不是只靠上层慢 PI。

## 3. 不采用的思路

本方案明确放弃以下思路：

1. 直接把 joint torque 当作轮压。
2. 直接让四个轮子的代理载荷始终相等。
3. 依赖 IMU 估计一个“全局坡度”，再据此推前后轮理想载荷。
4. 用单阈值判断“某轮悬空”或“底盘触地”。
5. 只靠上层慢速角度修正，不利用 ADRC 的快通道能力。

## 4. 总体策略

二代被动悬挂采用“四层结构”：

1. `Joint Support Observer`：从 joint、电机、车身和轮端信号估计每个角的广义支撑状态。
2. `Contact State Estimator`：融合多个证据，判断正常、轻载、冲击、打滑、触底嫌疑等事件。
3. `Passive Suspension Planner`：在 `common / pitch / roll / twist` 模态下生成被动悬挂目标，不追求绝对等载，而追求接地支撑余量和离地间隙。
4. `ADRC Fast Recovery Interface`：把 planner 生成的快速恢复命令映射成适合 ADRC 的角度、速度、加速度和扭矩前馈量，提高响应。

这四层的核心思想是：

1. 不先估“坡”，而是先估“支撑是否健康”。
2. 不先追求“均匀”，而是先避免“最轻轮掉下去”。
3. 不用全车平均值慢慢拉，而是给局部异常一个快通道。

## 5. 观测量与可利用信号

基于当前工程，二代方案优先使用以下信号：

1. joint 侧：
   - `physical_angle`
   - `physical_velocity`
   - `torque`
   - `target_physical_angle`
   - `target_physical_velocity`
   - `target_physical_acceleration`
   - joint 控制输出 torque
2. wheel 侧：
   - `wheel velocity`
   - `wheel torque`
   - `wheel control_torque`
   - `max_torque`
3. 车身侧：
   - `imu pitch`
   - `imu roll`
   - `imu pitch_rate`
   - `imu roll_rate`
   - 可选垂向冲击特征或高频角速度特征
4. 结构参数：
   - `rod_length`
   - `chassis_radius`
   - `wheel_radius`
   - `mass`
   - `moment_of_inertia`
   - `friction_coefficient`
   - joint 电机常数、减速比、最大力矩、ADRC 参数

## 6. 核心建模：广义支撑观测器

### 6.1 设计目标

观测器不直接输出“真实轮压”，而输出每个角的 `support_proxy` 与 `contact_margin`。

这两个量只需要满足：

1. 与真实接地支撑强弱单调相关。
2. 对 joint 角度、姿态、摩擦和惯性扰动尽量解耦。
3. 在不同工况下可比较、可过滤、可用于事件检测。

### 6.2 建议模型

对每个 joint，建立如下观测关系：

```text
tau_meas
= tau_support
+ tau_gravity(alpha, body_attitude)
+ tau_friction(alpha_dot)
+ tau_inertia(alpha, alpha_dot, alpha_ddot)
+ tau_bias(alpha)
+ tau_noise
```

在线估计量定义为：

```text
tau_support_hat
= tau_meas
- tau_gravity_hat
- tau_friction_hat
- tau_inertia_hat
- tau_bias_hat
```

再根据角度相关等效力臂归一化，得到：

```text
support_proxy = normalize(alpha, tau_support_hat)
```

### 6.3 为什么必须这样做

如果没有这一步：

1. 重载和上坡会被误判成某些轮异常重载。
2. 车身姿态变化会把重力项直接灌进 torque，误导均载控制。
3. joint 加减速时的惯性项会被误判成接地变化。
4. 低速摩擦、静摩擦翻转会制造大量假事件。

### 6.4 模型实现建议

第一版不要硬推一个完全解析的四连杆力学模型，而采用“解析骨架 + 标定表残差”的混合方法：

1. 解析骨架：用 joint 角度、车身姿态和基本几何推一个粗略重力项。
2. 标定表残差：在不同 `alpha / pitch / roll / alpha_dot` 区域标定静态和准静态偏差。
3. 在线残差滤波：对残差做慢速自适应，但只在可信窗口内更新。

可信窗口定义为：

1. joint 速度小。
2. wheel 滑移残差小。
3. 无强冲击。
4. 无触底嫌疑。
5. 四角接触置信度整体较高。

## 7. 非共面地形与模态分解

四个接触点不一定共面，这是二代方案必须显式建模的点。

被动悬挂 correction 必须分解成四个模态：

```text
common : 四角同向，用于整体抬升/降低、重载补偿、离地间隙恢复
pitch  : 前后差动，用于前后地形差
roll   : 左右差动，用于左右地形差
twist  : 对角差动，用于四点不共面地形
```

建议使用如下投影：

```text
m_common = (lf + lb + rb + rf) / 4
m_pitch  = (lf + rf - lb - rb) / 4
m_roll   = (lf + lb - rf - rb) / 4
m_twist  = (lf + rb - lb - rf) / 4
```

符号方向可在实车上按机构定义统一。

这里的核心结论是：

1. 平面坡可以主要由 `common + pitch + roll` 解释。
2. 斜着上坡、坡边界、台阶边、对角压缩地形必须依赖 `twist` 模态。
3. 如果控制器没有 `twist` 快通道，就无法在非共面地形里可靠地阻止单轮卸载。

## 8. 接触状态估计器

### 8.1 输出状态

对每个轮，估计：

1. `support_proxy`
2. `support_margin`
3. `contact_confidence`
4. `unload_confidence`
5. `impact_confidence`
6. `slip_confidence`
7. `travel_margin`

整车层面估计：

1. `bottom_contact_suspected`
2. `payload_state`
3. `geometry_state`

### 8.2 事件类型

建议至少分类以下事件：

1. `normal`
2. `local_unload`
3. `impact`
4. `wheel_slip`
5. `bottom_contact_suspected`
6. `overweight_or_common_sink`
7. `travel_saturation`

### 8.3 融合原则

每类事件不能只靠一个信号。

例如：

1. `local_unload`：
   - `support_proxy` 持续下降
   - `support_margin` 低于阈值
   - 可伴随 `wheel slip residual` 变大
   - 持续时间满足阈值
2. `impact`：
   - `d(torque)/dt` 尖峰很大
   - 持续时间短
   - 邻近轮不一定同步
   - 不一定伴随接触置信度持续下降
3. `bottom_contact_suspected`：
   - 多轮支撑模式异常
   - `common` 抬升命令效果变差
   - joint 已接近限位或 travel margin 很低
   - wheel residual 与 joint residual 同时异常
   - 可伴随 IMU 冲击特征

### 8.4 目标

接触状态估计器的目标不是给出绝对正确的二值判断，而是：

1. 让快恢复控制在真正需要时尽快触发。
2. 在小球、碎屑、局部冲击时少误触发。
3. 为保护逻辑提供可信的“嫌疑度”。

## 9. 重载、自重变化与离地间隙

“车身变重”不能靠瞬时 joint torque 直接判定，因为坡、姿态、接触状态都会影响 torque。

因此需要一个慢变化的 `payload_state`：

1. 只在低动态、无冲击、接触可信时更新。
2. 与 `common` 模态强绑定。
3. 更新后不直接改四轮均载目标，而是改“整车离地间隙目标”和“名义支撑流形”。

具体策略：

1. 当 `payload_state` 上升时，提高 `common` 基准，优先保留悬挂工作余量。
2. 当检测到 `common sink` 且并非局部卸载时，优先考虑“整体抬车”而不是“局部均载”。
3. 被动悬挂的第一约束不是均载，而是 `clearance >= clearance_min`。

## 10. 上坡、侧坡、坡边界

本方案不直接估“坡度”。

理由：

1. IMU 给的是车身姿态，不是地形姿态。
2. 车辆可能侧着上坡。
3. 车辆可能同时跨越多个坡面边界。
4. 四轮接触点可能根本不共面。

因此，本方案改为：

1. 用 IMU 只做重力补偿项估计，不做坡度主观测量。
2. 用四角 `support_proxy` 与 `contact_confidence` 描述当前支撑状态。
3. 用 `common / pitch / roll / twist` 模态自动吸收地形影响。

换句话说，本方案只回答“哪个模态失衡”，不回答“地面坡度是多少”。

## 11. 轮端打滑与地面碎屑

轮端打滑建议作为“辅助证据”，不是主观测量。

### 11.1 滑移残差

构造每个轮的 `slip residual`：

```text
r_slip_i = wheel_velocity_measured - wheel_velocity_predicted
```

其中 `wheel_velocity_predicted` 来自 chassis command、当前底盘 kinematics、当前等效半径和角速度命令。

同时可引入功率与轮力矩一致性特征：

```text
high wheel torque + low body response + abnormal wheel speed
=> slip_confidence increases
```

### 11.2 为什么不能直接拿来判触地

因为以下情况都会引发类似特征：

1. 轮子压到小球或碎屑
2. 地面局部很滑
3. 局部冲击导致轮速瞬态变化
4. 真正的轻载或悬空

所以 `slip_confidence` 只能和 `support_proxy`、`impact_confidence`、持续时间、模态残差一起融合。

## 12. 控制目标重定义

二代被动悬挂的数学目标建议写成：

```text
maximize min(contact_margin_i)
subject to:
    alpha_min <= alpha_i <= alpha_max
    travel_margin_i >= travel_margin_min
    clearance >= clearance_min
    |tau_cmd_i| <= tau_limit_i
```

这比“四轮等载”更符合真实需求。

含义如下：

1. 先保证最轻轮不要掉下去。
2. 再保证底盘不要触地。
3. 再保证 joint 不打到限位。
4. 再考虑整体均衡和平顺。

## 13. 快慢双通道控制

### 13.1 慢通道

慢通道负责：

1. 更新 `payload_state`
2. 更新 `nominal_support_manifold`
3. 调整 `common` 基准
4. 在稳定工况下慢慢修正长期偏差

这条通道可以使用：

1. 慢 PI
2. 低通观测值
3. 可信窗口自适应

### 13.2 快通道

快通道负责：

1. 识别 `local_unload`
2. 识别 `twist` 突发异常
3. 对轻载轮或轻载模态立即给恢复动作
4. 避免等均值误差慢慢积累

快通道的动作应该以模态或单角为单位生成：

1. `twist` 快恢复
2. `pitch` 快恢复
3. `roll` 快恢复
4. `common` 快抬车

快通道输出不能是“原始角度台阶”，而应当是平滑但激进的速度、加速度和前馈量。

## 14. ADRC 加速策略

这是本方案能否快于主动悬挂的关键。

### 14.1 现有 ADRC 的可利用点

现有 joint ADRC 已经具备：

1. `setpoint_angle`
2. `setpoint_velocity`
3. `TD`
4. `ESO`
5. `NLESF`

但如果只传角度或慢速速度目标，局部恢复不会快。

### 14.2 必须增加的输入

建议 joint controller 增加：

1. `setpoint_acceleration`
2. `feedforward_torque`

理由：

1. `setpoint_acceleration` 允许上层直接生成快恢复加速度，而不是让 TD 从角度阶跃慢慢追出来。
2. `feedforward_torque` 允许直接补偿重力项、支撑恢复项和冲击恢复项，减轻 ESO 的估计负担。

### 14.3 快恢复接口

当某个轮或某个模态被判为 `local_unload` 时，上层不再只改角度目标，而是同时给：

1. 一个短时速度参考脉冲
2. 一个短时加速度参考
3. 一个有符号 torque feedforward

这样 ADRC 的工作方式从：

```text
等误差变大再反馈纠正
```

改成：

```text
在误差刚出现时就主动注入结构化恢复动作
```

### 14.4 参数调度

建议对 ADRC 做事件触发调度：

1. 正常工况：保守参数，低噪声。
2. 快恢复工况：提高 TD 响应、提高 ESO 带宽、提高速度和加速度上限。
3. 冲击工况：短时抑制过激响应，防止追碎屑。

可以调度的参数包括：

1. `td_r`
2. `td_max_vel`
3. `td_max_acc`
4. `eso_w0`
5. `output limit`
6. `b0` 角度分段调度

### 14.5 软重置与预热

模式切换或事件切换时，建议支持：

1. TD 软重置
2. ESO 软重置或 warm start
3. 参考轨迹预热

避免在 `PASSIVE` 切入时因为内部状态不一致而丢掉第一拍响应。

## 15. 规划器逻辑

规划器不做“全局等载”，而做“状态机 + 模态控制”。

建议状态机：

1. `ObserveOnly`
2. `PassiveAssist`
3. `FastRecovery`
4. `ClearanceRecovery`
5. `Protection`

各状态含义：

1. `ObserveOnly`：只输出观测量，不动作，用于上车标定和日志阶段。
2. `PassiveAssist`：正常被动悬挂，做慢通道支撑整形。
3. `FastRecovery`：单轮卸载或 `twist` 异常时的快速恢复。
4. `ClearanceRecovery`：底盘触地嫌疑或 `common sink` 时，优先整体抬车。
5. `Protection`：触发限位、饱和、长期异常时降级，限制动作和速度。

## 16. 需要增加的参数

建议把参数分成六类：

1. 机构几何参数
2. 电机与 joint 力矩映射参数
3. 观测器补偿参数
4. 事件分类阈值与持续时间参数
5. 快慢通道控制参数
6. ADRC 调度参数

重点参数包括：

1. `support_model_*`
2. `gravity_bias_lut_*`
3. `friction_bias_*`
4. `impact_window_ms`
5. `unload_persistence_ms`
6. `slip_confidence_gain_*`
7. `clearance_recovery_gain_*`
8. `fast_recovery_velocity_limit_*`
9. `fast_recovery_acceleration_limit_*`
10. `adrc_fast_td_r`
11. `adrc_fast_eso_w0`
12. `feedforward_torque_limit_*`

## 17. 实施阶段

### P0：只采数，不控制

目标：建立 ground truth 数据集。

要做的事：

1. 增加 joint、wheel、IMU、command 全量日志。
2. 输出原始 `support_proxy_raw` 候选量。
3. 输出 wheel slip residual 候选量。
4. 设计标准测试工况。

标准工况建议包括：

1. 单轮垫高
2. 对角垫高
3. 斜着上坡
4. 坡边界切换
5. 低速过碎屑
6. 加重载荷
7. 底盘可控触地

### P1：观测器上线，但不闭环

目标：验证 `support_proxy` 与事件分类的可信度。

要做的事：

1. 实现 `Joint Support Observer`
2. 实现 `Contact State Estimator`
3. 输出观测量与置信度
4. 不对 joint 输出 correction

验收点：

1. 能区分 `impact` 与 `local_unload`
2. 能区分 `wheel_slip` 与 `local_unload`
3. 对斜着上坡和对角地形能看见 `twist` 模态异常

### P2：慢通道闭环

目标：上线 `PassiveAssist`，只做慢整形，不做快恢复。

要做的事：

1. 建立 `nominal_support_manifold`
2. 上线 `common / pitch / roll / twist` 慢通道
3. 上线 `payload_state` 慢更新

验收点：

1. 不引入明显抖动
2. 不因坡面误判而反向修正
3. 对长期不平地形有改善

### P3：快恢复通道 + ADRC 接口扩展

目标：实现局部恢复响应优于主动悬挂。

要做的事：

1. 给 joint ADRC 增加 `setpoint_acceleration`
2. 给 joint ADRC 增加 `feedforward_torque`
3. 实现事件触发快恢复
4. 实现 ADRC 参数调度

验收点：

1. 局部卸载检测到输出动作的时延不晚于主动悬挂
2. 对角卸载工况的 `twist` 恢复更快
3. 无明显误触发振荡

### P4：ClearanceRecovery 与 Protection

目标：处理触底嫌疑和饱和问题。

要做的事：

1. 上线 `bottom_contact_suspected`
2. 上线 `ClearanceRecovery`
3. 上线 joint 限位与扭矩饱和保护

验收点：

1. 底盘触地嫌疑时优先整体抬车
2. 不因误判碎屑导致频繁大动作
3. 限位附近不发散

## 18. 验收指标

建议使用以下量化指标：

1. `local_unload` 响应时延
2. `twist` 地形上的最小接触余量
3. 重载下的离地间隙保持能力
4. 底盘触地嫌疑场景的恢复时间
5. 小球/碎屑场景的误触发率
6. 在 `SPIN` 与 `STEP_DOWN` 并存时的稳定性

推荐把“优于主动悬挂”的定义写成明确指标，例如：

1. 在单轮快速卸载工况下，joint 快恢复动作起始时刻早于主动悬挂。
2. 在对角地形下，最轻轮接触余量的谷值高于主动悬挂。
3. 在误触发测试集中，快恢复误动作次数不高于设定阈值。

## 19. 风险与边界

本方案即使完整实现，仍然存在边界：

1. 没有直接轮压传感器时，`support_proxy` 永远只是代理量。
2. 没有底盘直接触地传感器时，`bottom_contact_suspected` 永远只能是高置信度推断。
3. 高动态剧烈机动下，观测器分离重力、摩擦、惯性和接触项会更困难。
4. 若机械工作空间本身不足，再好的控制器也无法保证始终四轮接地。

因此，若后续允许加少量低成本传感器，优先级建议为：

1. 底盘触地传感器
2. 轮端或连杆应变/力传感器
3. 更可靠的垂向冲击特征

## 20. 结论

二代被动悬挂的正确方向不是“做一个更快的四轮 torque 均值 PI”，而是：

1. 用机构模型、车身姿态和电机信息构造广义支撑观测器。
2. 用多源融合估计接触状态，而不是依赖单个 torque 或单个 IMU 量。
3. 用 `common / pitch / roll / twist` 模态处理非共面地形。
4. 用快慢双通道控制替代单通道慢均载。
5. 把 ADRC 的速度、加速度和扭矩前馈能力真正用起来，才有机会在局部卸载场景下比主动悬挂更快。

如果按这个 Plan 推进，第一阶段最重要的工作不是调控制参数，而是先做“观测器与事件分类的可信度建设”。只有观测器站得住，后续快恢复和被动悬挂闭环才值得上线。
