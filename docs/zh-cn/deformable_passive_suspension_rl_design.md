# 变形底盘被动悬挂强化学习方案

本文档定义一套面向变形底盘被动/半主动悬挂问题的强化学习方案，重点覆盖环境搭建、仿真搭建、状态与动作设计、奖励设计、与现有 ADRC 架构的结合方式，以及训练、验证、上车和安全策略。

本文档不主张一开始就让强化学习直接替代底层控制器，而是把强化学习作为上层参考修正策略，在保留现有 ADRC 关节伺服的前提下提升复杂接触工况下的表现。

## 1. 目标

强化学习方案的目标不是单纯追求四个 joint torque 相等，也不是盲目让车身姿态绝对水平，而是在复杂地形和复杂接触条件下改善以下能力：

1. 车身姿态稳定性。
2. 四角支撑合理性与接地连续性。
3. 触底风险控制。
4. 坡道、坡边、斜着上坡和障碍通过性。
5. 碎屑、小球、短时冲击下的鲁棒性。
6. 与现有底层 ADRC 控制器的兼容性与安全性。

## 2. 当前工程基础

结合当前仓库实现，底盘侧已经具备较完整的观测与执行链：

1. 上层底盘控制器可读取：
   - 四个 joint physical angle。
   - 四个 joint torque。
   - chassis IMU pitch / roll / pitch_rate / roll_rate。
   - wheel velocity。
2. 上层可生成：
   - joint target angle。
   - joint target velocity。
   - joint target acceleration。
3. 每个 joint 下层使用 ADRC 做角度伺服。
4. 当前 passive suspension 的核心逻辑是：
   - `joint torque -> load proxy -> 四角均载 PI correction`。

这意味着强化学习方案不需要从零构建整条控制链，而是应建立在当前观测量、轨迹生成和 ADRC 执行链基础上。

## 3. 为什么考虑强化学习

这个问题适合强化学习的原因是：

1. 地形接触强非线性。
2. 触地、脱地、压碎屑、跨坡边界都属于非平滑事件。
3. 平行四边形机构下，joint torque 与 wheel load 的关系受 joint angle 和姿态强耦合。
4. 单阈值和简单规则难以覆盖复杂工况。
5. 控制目标是长期累计表现更优，而不是只在某一时刻瞬时最优。

但强化学习不是魔法。它依赖：

1. 足够可信的仿真环境。
2. 足够合理的状态设计。
3. 严格的安全包络。
4. 良好的 sim-to-real 策略。

## 4. 总体控制架构

推荐采用分层控制，而不是端到端直接输出电机扭矩。

控制结构建议为三层：

1. 高层：强化学习策略。
   - 输入：观测状态。
   - 输出：悬挂参考修正量。
2. 中层：悬挂参考生成器。
   - 将强化学习输出转成 joint target angle / velocity / acceleration correction。
3. 低层：现有 ADRC joint controller。
   - 负责稳定、快速地执行参考轨迹。

推荐的参考组合关系为：

```text
joint_target_final
= joint_target_nominal
+ delta_target_from_classical_observer
+ delta_target_from_rl
```

其中：

1. `joint_target_nominal` 是原始姿态目标。
2. `delta_target_from_classical_observer` 是传统观测器或保护逻辑输出。
3. `delta_target_from_rl` 是强化学习输出的修正量。

第一阶段不建议让强化学习直接输出 joint motor torque。

## 5. 强化学习环境定义

### 5.1 环境类型

环境定义为连续控制环境：

1. observation space：连续向量。
2. action space：连续向量。
3. reward：逐步累计。
4. episode：一个完整通过或扰动过程。

推荐环境接口兼容 Gymnasium 风格，便于后续接入 PPO、SAC 等算法。

### 5.2 单步流程

每一步环境推进建议遵循：

1. 读取仿真或中间控制层当前状态。
2. 将状态构造成 observation。
3. 策略输出 action。
4. 中层将 action 映射为 joint correction reference。
5. 下层 ADRC 根据 joint 参考生成控制输出。
6. 仿真推进一个时间步。
7. 计算 reward、done、info。

## 6. 状态设计

状态必须同时反映当前姿态、关节状态、轮端状态和接触历史。

### 6.1 基础状态

每步状态建议至少包含：

1. 四个 joint angle。
2. 四个 joint velocity。
3. 四个 joint torque。
4. IMU pitch。
5. IMU roll。
6. IMU pitch_rate。
7. IMU roll_rate。
8. 四个 wheel velocity。
9. 底盘控制命令：
   - `vx command`
   - `vy command`
   - `wz command`
10. 当前 nominal joint target。
11. 当前 nominal joint target velocity。
12. 当前姿态模式标志。
13. 每个 joint 到上下限的 margin。

### 6.2 扩展状态

建议逐步加入这些更高价值的中间量：

1. 归一化 torque residual。
2. 基于机构模型估计的 normalized load proxy。
3. slip indicator。
4. bottom-out risk estimate。
5. correction gate / confidence。
6. chassis power / torque margin。
7. payload estimate。

### 6.3 历史信息

只看单帧观测通常不够，建议补充时间信息，二选一或两者结合：

1. 堆叠最近 N 帧观测。
2. 使用 RNN / GRU policy。

第一版可先取最近 5 到 20 帧作为堆叠输入。

历史信息重要的原因是：

1. 碎屑冲击和真实支撑变化的差别主要体现在时间结构上。
2. 坡边、斜坡和跨边界工况需要通过动态过程识别。
3. 单步观测不一定足以辨识地形状态。

## 7. 动作设计

### 7.1 推荐动作形式

最推荐的动作不是四个 joint 直接 torque，而是四个 joint 的参考修正量：

1. `delta_theta_lf`
2. `delta_theta_lb`
3. `delta_theta_rb`
4. `delta_theta_rf`

也可以输出更结构化的低维动作：

1. 前后俯仰修正量。
2. 左右横滚修正量。
3. 整体高度偏置。
4. 四角均载偏置。

然后由中层映射到四个 joint。

### 7.2 动作层级建议

按风险从低到高排序：

1. 输出四个 joint angle correction。
2. 输出 joint target velocity correction。
3. 输出 joint target acceleration correction。
4. 输出 torque feedforward。
5. 直接输出 motor torque。

第一阶段建议仅做：

1. angle correction。
2. 或 angle + velocity correction。

### 7.3 动作约束

必须对动作施加以下约束：

1. 角度限幅。
2. 速度限幅。
3. 加速度限幅。
4. 平滑约束。
5. 与 joint 上下限相关的 barrier 或 soft constraint。

## 8. 奖励设计

奖励必须同时反映稳定、通过、安全和平顺。

### 8.1 主奖励项

建议包含以下部分：

1. 姿态稳定：
   - 惩罚 `|pitch|`。
   - 惩罚 `|roll|`。
2. 姿态动态平顺：
   - 惩罚 `|pitch_rate|`。
   - 惩罚 `|roll_rate|`。
3. 载荷均衡：
   - 惩罚 four-corner load variance。
4. 通过性：
   - 奖励 chassis velocity tracking。
5. 触底控制：
   - 惩罚 bottom-out event。
   - 惩罚 sustained bottom-out risk。
6. 打滑控制：
   - 惩罚 slip risk。
7. 动作平滑：
   - 惩罚 action magnitude。
   - 惩罚 action rate。
8. 执行安全：
   - 惩罚 joint hitting limits。

### 8.2 推荐奖励形式

可采用如下形式：

```text
reward =
- a1 * abs(pitch)
- a2 * abs(roll)
- a3 * abs(pitch_rate)
- a4 * abs(roll_rate)
- a5 * load_variance
- a6 * bottom_out_penalty
- a7 * slip_penalty
- a8 * action_norm
- a9 * action_rate_norm
- a10 * joint_limit_penalty
+ b1 * velocity_tracking_score
+ b2 * terrain_pass_success_bonus
```

### 8.3 奖励设计注意事项

1. 不要只有姿态奖励，否则策略可能学成“僵住不动”。
2. 不要只奖均载，否则可能牺牲通过性。
3. 不要只奖速度跟踪，否则可能学会过度压悬挂。
4. 奖励权重建议配合 curriculum 逐步调整。

## 9. 仿真环境搭建

### 9.1 仿真目标

仿真环境至少需要还原：

1. 平行四边形关节机构几何。
2. 四角轮地接触。
3. chassis 刚体动力学。
4. IMU 观测。
5. joint torque / angle / velocity 反馈。
6. wheel velocity 反馈。
7. 多种地形和非理想接触。

### 9.2 推荐仿真平台

按优先级建议：

1. MuJoCo。
   - 接触稳定。
   - 连续控制成熟。
   - RL 社区支持好。
   - 适合快速迭代。
2. Isaac Lab / Isaac Sim。
   - GPU 并行训练强。
   - 大规模 randomization 方便。
   - 适合后期大规模训练。
3. Gazebo / Ignition。
   - 更适合 ROS 联调。
   - 纯 RL 训练效率通常不如前两者。

### 9.3 推荐路线

建议路线：

1. 第一阶段使用 MuJoCo 建模，把控制问题先跑通。
2. 第二阶段若追求大规模并行训练，再迁移到 Isaac Lab。
3. 第三阶段通过 ROS 或现有 RMCS 接口做联调和回放验证。

## 10. 仿真模型必须包含的物理要素

### 10.1 机构建模

必须包含：

1. 车体主刚体。
2. 四个平行四边形升降机构。
3. 四个轮组。
4. joint transmission geometry。
5. joint mechanical limits。
6. motor-side equivalent inertia / damping / friction。

### 10.2 接触建模

训练环境至少覆盖：

1. 平地。
2. 单侧上坡。
3. 斜着上坡。
4. 多坡边界。
5. 小障碍。
6. 碎屑 / 小球类短时接触。
7. 摩擦系数变化。
8. 全向轮近似接触与侧滑特性。

### 10.3 传感器建模

应仿真以下非理想因素：

1. IMU 偏置。
2. IMU 噪声。
3. IMU 延迟。
4. joint torque noise。
5. joint angle quantization 或 delay。
6. wheel velocity noise。

## 11. 训练场景设计

训练场景不能只用平地和标准坡道。

### 11.1 场景库

建议训练集包含：

1. 平地直行。
2. 小幅随机起伏。
3. 单前轮压障碍。
4. 单后轮压障碍。
5. 对角轮压障碍。
6. 前轮先上坡后车体过渡。
7. 斜着上坡。
8. 坡边切换。
9. 松散碎屑接触。
10. 低摩擦地面。
11. 随机载荷变化。
12. 不同 joint 初始姿态。

### 11.2 课程学习

推荐采用 curriculum learning：

1. 第 1 阶段：平地 + 小起伏。
2. 第 2 阶段：标准坡道 + 单障碍。
3. 第 3 阶段：斜坡 + 坡边。
4. 第 4 阶段：碎屑 + 摩擦变化 + 随机载荷。
5. 第 5 阶段：全混合场景。

## 12. Domain Randomization

为了减小 sim-to-real gap，训练时应随机化：

1. 车体质量。
2. 重心高度。
3. 关节摩擦。
4. 电机有效输出常数。
5. 接触摩擦系数。
6. 地形高度扰动。
7. IMU 噪声与偏置。
8. torque bias。
9. actuator delay。
10. actuator saturation。
11. 轮地滚动阻力。
12. 小球 / 碎屑尺寸、硬度、分布。

注意：随机化不应一上来就过强，否则训练容易学不动。建议随 curriculum 逐步加大。

## 13. 强化学习算法选择

### 13.1 推荐算法

优先考虑：

1. PPO。
   - 稳定。
   - 工程成熟。
   - 容易做大规模并行训练。
2. SAC。
   - 连续动作样本效率高。
   - 适合精细控制。
3. RNN PPO 或 recurrent SAC。
   - 当历史信息非常关键时再考虑。

### 13.2 初期建议

第一阶段建议直接用 PPO：

1. 实现成熟。
2. 便于调试。
3. 容易与 curriculum 和 randomization 结合。
4. 更适合先做 baseline。

## 14. 与现有 ADRC 的结合方式

### 14.1 不建议的方式

不建议：

1. 让 RL 直接输出 motor torque。
2. 直接替代现有 joint ADRC。
3. 一开始就做从 IMU 到 torque 的端到端控制。

原因是：

1. 风险高。
2. 训练难。
3. sim-to-real 差距大。
4. 缺少安全包络。

### 14.2 推荐方式

保留下层 ADRC，仅让 RL 输出：

1. joint angle correction。
2. joint velocity correction。
3. 可选的 target acceleration correction。

即：

```text
joint_ref = nominal_ref + rl_correction
```

ADRC 继续负责：

1. tracking。
2. disturbance rejection。
3. actuator-side stabilization。

### 14.3 Residual RL

推荐后续采用 residual RL：

```text
u_final = u_classical + delta_u_rl
```

其中 `u_classical` 可以是：

1. 当前 passive suspension 逻辑升级版。
2. 基于几何、姿态和载荷残差的 classical observer 输出。

这样做的好处是：

1. 学习难度更低。
2. 有 baseline 托底。
3. 安全性更高。
4. 更容易解释 RL 到底学到了什么。

## 15. 安全机制

RL 上车前必须设计安全层。

### 15.1 训练时安全

训练期至少应包含：

1. action clipping。
2. joint limit penalty。
3. velocity / acceleration envelope。
4. unstable episode early termination。
5. chassis overturn 或 impossible contact termination。

### 15.2 部署时安全

部署期必须增加 runtime safety shield：

1. correction amplitude clamp。
2. correction rate clamp。
3. joint limit guard。
4. confidence gate。
5. abnormal torque spike suppressor。
6. fallback to classical mode。

### 15.3 回退逻辑

任一情况都应回退到 classical controller：

1. RL 输出 NaN。
2. IMU 或 torque 数据失真。
3. joint 接近危险极限。
4. 出现高频振荡。
5. watchdog 触发。
6. RL confidence 过低。

## 16. 训练流程

### 16.1 数据准备

虽然 RL 不一定必须先有离线数据，但建议先采集真车数据用于：

1. 参数辨识。
2. 几何与摩擦模型校准。
3. 传感器噪声模型估计。
4. 验证仿真分布是否合理。

### 16.2 训练流程

建议训练流程为：

1. 建立机械与接触仿真。
2. 接入 nominal controller + ADRC。
3. 先训练一个不输出修正的 baseline。
4. 加入 classical observer。
5. 再训练 RL residual policy。
6. 加 curriculum + randomization。
7. 记录每类场景 success metrics。
8. 做 ablation study。

### 16.3 验证指标

至少统计以下指标：

1. 平均姿态误差。
2. 姿态角速度 RMS。
3. 触底次数。
4. slip 次数。
5. 速度跟踪误差。
6. joint hitting limit 次数。
7. action smoothness。
8. terrain pass rate。

## 17. Sim-to-Real 路线

### 17.1 上车前验证

上车前建议分三层验证：

1. 纯仿真随机场景验证。
2. 仿真回放真实日志条件验证。
3. 软件在环或半实物验证。

### 17.2 上车阶段

部署建议分阶段进行：

1. shadow mode。
   - RL 只输出建议，不接管。
2. gated mode。
   - 小幅修正，严格限幅。
3. limited authority mode。
   - 只在特定模式下启用。
4. full residual mode。
   - 仍保留 fallback。

### 17.3 必须记录的量

部署时建议持续记录：

1. observation vector。
2. RL action。
3. action after safety shield。
4. joint target / final target。
5. joint torque。
6. IMU。
7. wheel velocity。
8. fallback events。
9. bottom-out indicators。
10. slip indicators。

## 18. 项目实施路线图

### 阶段 A：建模与 baseline

目标：

1. 建立几何模型。
2. 校准 joint torque 与姿态、角度耦合。
3. 建立仿真底盘。
4. 复现当前 passive suspension baseline。

产出：

1. 仿真环境 v1。
2. 参数辨识文档。
3. baseline 指标。

### 阶段 B：classical observer 升级

目标：

1. 构建 normalized load proxy。
2. 构建 torque residual。
3. 构建 contact confidence / slip risk / bottom-out risk。
4. 形成更可靠的 classical controller。

产出：

1. classical suspension v2。
2. 可解释的中间状态量。

### 阶段 C：residual RL

目标：

1. RL 输出小幅 correction。
2. 在多地形上超过 classical baseline。
3. 建立安全包络和回退逻辑。

产出：

1. RL residual policy v1。
2. 与 classical baseline 对比报告。

### 阶段 D：真实车验证

目标：

1. shadow mode。
2. gated mode。
3. limited authority deployment。

产出：

1. 真车测试报告。
2. sim-to-real gap 分析。
3. 下一阶段迭代建议。

## 19. 关键风险

### 19.1 物理建模不足

如果平行四边形机构和接触建模不对，RL 学到的策略很可能不可迁移。

### 19.2 状态不充分

如果观测不能表达真实支撑变化，策略容易学成脆弱规则。

### 19.3 奖励设计失衡

如果奖励只关注姿态，策略可能牺牲通过性；如果只关注通过性，策略可能学出过度冲击动作。

### 19.4 sim-to-real 差距

碎屑、小球、坡边等问题最容易造成仿真与真实的分布差异。

### 19.5 安全问题

没有 runtime safety shield，就不应直接上车。

## 20. 当前最推荐的执行顺序

1. 先做几何与载荷映射建模。
2. 建立 MuJoCo 仿真环境。
3. 先复现当前 passive baseline。
4. 升级 classical observer。
5. 再做 residual RL。
6. 最后再讨论是否扩大 RL 权限。

## 21. 结论

对于变形底盘被动悬挂问题，强化学习最合适的角色不是直接替代底层关节控制，而是作为：

1. 面向复杂接触工况的高层参考修正策略。
2. 建立在 classical controller 基础上的 residual optimizer。
3. 在安全包络内提升复杂地形表现的工具。

短期最现实路线是：

1. classical baseline 先做强。
2. 仿真先做可信。
3. RL 先做 residual。
4. ADRC 保留为底层执行器。

这条路线在工程上更稳，也更容易真正落地到真实车。
