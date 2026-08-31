# deformable-b 主动悬挂 RL 部署验证清单

本文档记录 deformable-b（omni-b 变形底盘）上 rmcs_rl 部署的设计决策、改动清单与分阶段验证步骤。

## 1. 设计决策（已定稿）

- **RL 替代范围**：`DeformableSuspension`（姿态外环 PID）+ 4×`DeformableJointController`（ADRC 内环）。
  输入：底盘 IMU（姿态/角速度）+ 4 关节反馈 → 策略 → 4 关节力矩。
  **轮速底盘控制（DeformableChassis + DeformableOmniWheelController）保持传统控制，不进策略。**
- **策略合同（22/4）**：`obs = cmd3 | height1 | angvel3 | gravity3 | pos4 | vel4 | act4`（10+3×4=22），
  `actions = 4` 位置 PD 目标（action_scale 0.25）。走 RlController 现有构建器，只改参数。
- **路线 A（已定稿）**：单策略 + 高度指令条件化，覆盖三档基准——
  普通 `height=0.132`（基准角 = default_dof_pos，四腿同角）/ 高 `0.16~0.17`
  （超物理上限 → 策略饱和到机械极限 = 尽量高）/ 低 `0.05~0.06`（≈ 机械下限 = 尽量低）。
  关节限位 0~57° 对应车高 [0.045, 0.132] m 全程有效；
  训练 `height_range=[0.05, 0.17]` 与部署指令范围严格对齐；
  车体触地由接触力终止兜底（高度终止 0.03 仅防数值异常）。
- **推理频率 100Hz**（训练 decimation 2 × dt 0.005）。
- **DOF 序**：`left_front_joint, left_back_joint, right_back_joint, right_front_joint`（与硬件 kJointName 一致）。
- **关节角语义**：训练/部署统一用 physical 角（`physical = 62.5° − 电机角`）。

## 2. 改动文件清单

### RMCS 侧（分支 deformable_test_RL）
| 文件 | 改动 |
|---|---|
| `rmcs_ws/src/rmcs_rl/src/rl_controller.cpp` | 新增分组接口后缀参数（position/velocity 组各自 angle/velocity 后缀，默认 `/angle`、`/velocity`，向后兼容） |
| `rmcs_ws/src/rmcs_rl/src/rl_debug_command.cpp` | 新增：订阅 topic 写 command 接口的调试指令源组件 |
| `rmcs_ws/src/rmcs_rl/plugins.xml` / `package.xml` | 注册 RlDebugCommand / 增加 std_msgs 依赖 |
| `rmcs_ws/src/rmcs_rl/tool/gen_synthetic_policy.py` | synthetic ONNX 生成（合同校验用） |
| `rmcs_ws/src/rmcs_rl/tool/check_policy_contract.py` | ONNX 合同检查（名字/形状/dtype/有限性） |
| `rmcs_ws/src/rmcs_rl/models/policy.onnx` | synthetic 零动作策略（obs[1,22]→actions[1,4]） |
| `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni-b.cpp` | 新增 `/chassis/imu/quaternion` + `/chassis/imu/angular_velocity` 输出（与 wheel-leg 约定一致） |
| `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-b-rl.yaml` | 验证配置（仅 hardware + RlController + RlDebugCommand + ValueBroadcaster） |

### 训练侧（~/legged_gym，任务 deformable_suspension）
| 文件 | 说明 |
|---|---|
| `legged_gym/envs/deformable_suspension/deformable_suspension.py` | 4 腿主动悬挂 env（轮无驱动、平四耦合、22 维观测合同） |
| `legged_gym/envs/deformable_suspension/deformable_suspension_config.py` | 配置（rough 地形课程、高度指令随机、奖励 = 水平 + 四轮着地 + 高度 + 平滑） |
| `legged_gym/envs/__init__.py` | 注册 `deformable_suspension` 任务 |

## 3. 待标定项（真机，上电后必须完成）

- [ ] `default_dof_pos`：水平站立时 4 腿 physical 角实测值（替换 yaml 中的 0.0）
- [ ] `prepare_dof_pos`：同站立位姿
- [ ] `position_torque_max`：按 LK 电机力矩量纲标定后收紧（当前 20.0 为保守值）
- [ ] 轮速度符号 / IMU 安装方向与训练坐标系一致性复核（`set_coordinate_mapping`）
- [ ] `command_height` 语义（车体原点高度）与训练 `height_range` [0.05, 0.17] 对齐；三档基准值：0.132 / 0.16+ / 0.05~0.06

## 4. 验证步骤

### L0 离线合同验证（无硬件）
```bash
# 已生成：rmcs_rl/models/policy.onnx（零动作策略）
python3 tool/check_policy_contract.py models/policy.onnx --obs 22 --act 4        # 应通过
python3 tool/check_policy_contract.py /tmp/policy_wrong_contract.onnx --obs 22 --act 4  # 应 FAIL
```

### L0.5 管线级测试（无硬件，RlFakePlant 仿真桩）—— 已验证通过 ✅
用 `deformable-infantry-omni-b-rl-fake.yaml`（RlFakePlant 模拟硬件接口 + RlController + RlDebugCommand），
在 devcontainer 内跑完整链路：模型加载 → 接口绑定 → FSM → 观测构建 → PD → 力矩：

```bash
# 构建后
source install/setup.bash
ros2 run rmcs_executor rmcs_executor --ros-args \
    --params-file src/rmcs_bringup/config/deformable-infantry-omni-b-rl-fake.yaml
# 另一终端驱动/观测：
ros2 topic pub -1 /chassis/debug/command std_msgs/msg/Float64MultiArray "{data: [0,0,0.132,2]}"  # PREPARE
ros2 topic pub -1 /chassis/debug/command std_msgs/msg/Float64MultiArray "{data: [0,0,0.132,3]}"  # RL
ros2 topic pub -1 /chassis/debug/reset std_msgs/msg/Bool "{data: true}"                          # 复位
ros2 topic echo /chassis/fake/state /chassis/fake/observation /chassis/fake/torques
```
已验证项：状态机 0→2→3（未 PREPARE 前 3 被拒）、静止水平时观测 gravity=(0,0,−1)、
height=0.66、关节段≈0、零动作策略下力矩≈0、reset 回 IDLE、错误合同模型拒绝加载。

**踩坑记录**：
1. **rcl 参数文件不支持空列表** `[]`（落盘为 PARAMETER_NOT_SET，auto-declare 抛异常）——
   `velocity_pd_joints: []` 已从配置移除（代码默认空组）；
2. **rmcs_executor 接口图要求无环**——仿真桩必须用 Status/Command partner 双组件架构
   （`create_partner_component`，与真实硬件同构），否则 Circular dependency 拒绝启动；
3. 调试观测请用 `ros2 topic echo` 原始输出，勿用 `tr -d " -"` 解析（会吞负号，e-16 显示成 e16）。

### L1 构建 + 启动 + 状态机（台架，腿支撑轮悬空）
```bash
build-rmcs                                   # devcontainer 内
set-robot deformable-infantry-omni-b-rl && launch-rmcs
# 指令源（另一终端）：
ros2 topic pub /chassis/debug/command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.132, 2.0]}"  # 进 PREPARE
ros2 topic pub /chassis/debug/command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.132, 3.0]}"  # 进 RL（未 PREPARE 完成会被拒绝）
ros2 topic pub /chassis/debug/reset std_msgs/msg/Bool "{data: true}"              # 干净复位
```
检查点：
- [ ] 启动日志：接口绑定全绿、模型加载成功（obs[1,22]→actions[1,4]）
- [ ] state 3 在未 PREPARE 完成时被拒绝（日志 `Refusing RL`）
- [ ] 静止时观测：gravity≈(0,0,-1)、关节段=physical−default、力矩=0（IDLE）
- [ ] PREPARE 慢速插值到预备位；RL + 零动作 → PD 保持
- [ ] 拔板卡 USB → obs NaN → failSafe 回 IDLE 零力矩；reset_count++ → 干净复位

### L2 台架电机级（腿受力、轮悬空）
- [ ] 零动作策略下腿 PD 抗重力保持（对照 kp=200/kd=4）
- [ ] LK `temperature` 无过热趋势；力矩在 `position_torque_max` 内

### L3 地面递进（真实策略导出后）
1. [ ] 站立（height=0.132）30s 无漂移振荡
2. [ ] 高度指令 0.132 → 0.16（高基准，应饱和到机械极限）→ 0.05~0.06（低基准，≈ 机械下限）升降跟踪
3. [ ] 粗糙地形/斜坡上手推扰动 → 自动恢复水平、四轮着地
4. [ ] 30 分钟 soak（温度、漂移、failSafe 无误触发）

## 5. 训练与导出（GPU 恢复后，训练栈 = Isaac Lab）

训练栈已从 Isaac Gym/legged_gym 切换到 **Isaac Sim 5.1 + Isaac Lab 2.3 + RSL-RL**
（wheeled-legged_RL 仓库，isaaclab conda 环境），任务为 `Robotics-Deformable-Suspension(-Rough)-v0`：

```bash
# 0) 一次性：URDF → USD（需 GPU）
bash ~/Documents/workspace/wheeled-legged_RL/scripts/tools/convert_deformable_urdf.sh

# 1) 训练（平面快速验证 / 粗糙地形正式训练）
conda activate isaaclab
cd ~/Documents/workspace/wheeled-legged_RL
python scripts/rsl_rl/train.py --task=Robotics-Deformable-Suspension-v0 \
    --num_envs=4096 --max_iterations=20000 --headless --device=cuda:0
python scripts/rsl_rl/train.py --task=Robotics-Deformable-Suspension-Rough-v0 \
    --num_envs=4096 --max_iterations=20000 --headless --device=cuda:0

# 2) 导出（rsl_rl exporter：input "obs" / output "actions" float32）
# 3) 合同核对 + 替换 rmcs_rl/models/policy.onnx
python3 rmcs_rl/tool/check_policy_contract.py policy.onnx --obs 22 --act 4
```

新增/修改文件（wheeled-legged_RL 仓库）：
- `source/agent_tasks/agent_tasks/direct/deformable_suspension/{env.py, env_cfg.py, __init__.py, agents/rsl_rl_ppo_cfg.py}`
- `source/agent_world/agent_world/assets/deformable_suspension.py`
- `scripts/tools/convert_deformable_urdf.sh`
- `source/agent_tasks/agent_tasks/__init__.py`（注册导入）

注意：合同/参数与部署逐项同构（obs 22、act 4、kp/kd=200/4、height_range [0.05,0.17]、
100Hz）；平四耦合用虚拟刚弹簧（k=1000/d=10）；轮子零驱动。
原 legged_gym 版 env（`~/legged_gym` 的 deformable_suspension）保留但不再使用。

## 6. 注意

- 台架测试轮子悬空时 velocity PD 组会空转——本设计轮子不进策略，无此风险；但底盘轮控组件也未挂载（验证配置无 /remote/* 生产者），轮子处于无指令状态。
- `~/legged_gym/logs/deformable_suspension/` 训练日志；GPU 当前不可用（nvidia-smi 失败），训练需先恢复驱动。
