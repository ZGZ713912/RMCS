# deformable 关节 ADRC 与电流计数扫频

deformable 的四个 joint 继续使用 TD + ESO + NLESF 的 ADRC。正常运行的四份
`deformable-infantry-omni{,-b,-c,-rl}.yaml` 和新扫频配置均使用 `output_domain: current_raw`。
关节控制链现在是：

```text
physical_angle / physical_velocity
  -> ADRC -> servo current counts
  -> 加扫频探针 -> 最终 CAN 计数饱和/取整 -> 0xA1
```

正常配置没有探针，ADRC 直接输出 `/chassis/<joint>/control_current_raw`。
实验配置使用独立 `DeformableCurrentSweep` 插件，ADRC 输出 `/servo_current_raw`，
插件的 command partner 相加、编码并记录。旧通用 `SweptFrequencyController` 和
`fit_sweep_graybox.py` 保留给使用其他协议/已有扭矩标定的实验；deformable 应使用这里的新配置。

## 已移除的限幅与保留的边界

- 当前模式不再读取、应用 NLESF 的 `u_min/u_max` 或末级的 `output_min/output_max`。
  四份正常配置和两份实验配置均已删除这些参数。也不按 25 N·m 额定峰值裁剪扫频输出。
- `controller_output_to_current_raw: 5.74635241301908` 保留旧配置**未饱和时**的 CAN
  计数增益，便于沿用现有 `b0/k1/k2/kt`。它只是历史控制单位到计数的缩放，不能作为
  已标定的 N·m 系数。之后更改厂家 Kt 不会改变这个原始计数控制路径。
- ESO 使用上一条 `/command_current_raw` 除以该缩放，包含探针、取整和最后饱和的结果。
  它不再用未施加的 ADRC 请求代替实际编码指令。这里的“实际”是已准备的 CAN 数据，
  并不证明 USB/CAN 总线成功交付。
- 唯一默认的输出幅值边界是协议 `±2048` 计数。`max_current_slew_raw_s: 0.0`
  表示没有额外电流变化率限制。需要较低电流边界时，可明确设置计数单位的
  `max_command_current_raw` 和硬件 `joint_current_command_limit`；二者均不依赖 N·m。
- 物理角度、速度、温度、反馈超时、异常计数、执行周期中断和定位/稳定超时仍会停止实验。
  默认物理角度边界取**当前源 YAML** 的 17–75°。探针阶段允许相对保持点偏移 0.14 rad，
  并受上述全局边界约束。TD 的轨迹约束和 ESO 的数值状态约束不属于扭矩限幅。

`/raw_current` 是 CAN 帧里的 iq 原始值；`/current_raw` 已按 `set_reversed()` 变换至
驱动配置坐标。`/command_current_raw` 使用相同坐标。当前四个关节的物理腿角为
`73.31° - motor_angle`，所以电流到物理角的方向为 `-1`。实验使用物理腿角，不使用 RL 缩放角。
原 `/torque` 仍是未标定的电流换算代理；既不参与该扫频控制，也不作为本拟合的 N·m 标签。
旧策略的观测接口没有在此改成计数，使用新模型训练/部署时需同步更新策略契约。

## 配置、开始与停止

预置配置：

- `rmcs_ws/src/rmcs_bringup/config/deformable-current-sweep-suspended.yaml`：悬空条件。
- `rmcs_ws/src/rmcs_bringup/config/deformable-current-sweep-grounded.yaml`：落地条件。

它们复制当前 RL YAML 的实际硬件插件、板卡序列号、标定和四路 ADRC 增益，仅装配关节实验链。
其他电机的未绑定输入生成零/停机指令。修改源配置后重新生成：

```bash
python3 .script/identification/prepare_current_sweep.py --condition suspended
python3 .script/identification/prepare_current_sweep.py --condition grounded
```

用 `--source` 选择 A/B/C 车辆的实际源配置；不要跨车复用板卡序列号。
`--angles-deg` 和 `--amplitudes-raw` 可以直接设置覆盖点和计数幅值，不需要扭矩常数。

默认程序访问 65、50、35、25、35、50、65°，每点使用 128 和 384 计数，
逐关节施加 20 s 的 0.2–8 Hz chirp，稳定后再施加 6 s 正负阶跃作为独立验证。
全部有 56 组 chirp/验证块，含稳定和移动约需半小时；实际时长取决于 ADRC 能否稳定及温度。
幅值、频率、角度和温度覆盖由真实记录确认，默认配置本身不证明足够激励。

在用户确认实验布置、完成正常编译/安装后选择相应配置启动进程。例如：

```bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=deformable-current-sweep-suspended
ros2 service call /identification/status std_srvs/srv/Trigger '{}'
ros2 service call /identification/start std_srvs/srv/Trigger '{}'
ros2 service call /identification/stop std_srvs/srv/Trigger '{}'
```

进程启动和进程重启都不会自动开始实验；start 只是排队，状态服务给出反馈检查结果、阶段和文件路径。
`require_remote_enable: false` 允许无遥控器的显式 start，遥控器双下仍会复位并归零。
stop、故障、完成均将四路电流请求归零并清空 ADRC 状态；重新开始须再次显式 start。
停止时后台线程自行排空 CSV 队列，不等待磁盘写完才准备零电流帧。

本次代码修改未在真车上启动、部署或自动运行上述命令。

## 采样与数据有效性

每次开始创建 `/tmp/deformable_current_sweep_<condition>_<host-time>.csv`，并生成两个 sidecar：

- `.csv.json`：计数单位、关节顺序、方向、工况、生成时的硬件/ADRC/扫频参数快照。
  手工修改或启动时覆盖配置后，应重新生成/保存实验配置，避免快照与实际设置不一致。
- `.csv.status.json`：后台写完后生成；包含完整完成标志、写入失败及丢行数量。

CSV 同时保存四个关节的物理角/速度、iq 计数、温度、反馈序号及接收时间、
上一条已编码电流/指令序号/准备时间、ADRC 请求、探针、求和请求、最终计数、
幅值/变化率饱和标志、物理参考和机体 roll/pitch。

阶段编号为 idle=0、move=1、settle=2、chirp=3、validation=4、complete=5、aborted=6。
速度/位置辨识仅使用 3，独立验证仅使用 4。CSV 行率与反馈率不同：四个关节 CAN 分时发送，
重复反馈必须按序号去重。时间戳来自主机单调时钟；准备时间不是总线发送时间，接收时间不是
电机内部采样时间。拟合得到的延迟包含主机传输与反馈链的影响，精度受采样周期制约。

队列丢行、写盘失败会中止实验；缺失 status sidecar 的文件可能仍在写入或已异常中断。
正常拟合要求实验完整完成、无丢行、无写入失败。`--allow-partial` 只生成诊断结果，
始终不能获得 `training_ready: true`。

## 拟合和 real2sim 使用边界

```bash
python3 .script/identification/fit_current_sweep.py /tmp/实际文件.csv --output /tmp/suspended-model.json
```

需要 NumPy、SciPy；配置生成/配置测试另需 PyYAML。拟合按关节和工况输出模型 JSON 和同名 NPZ，
不会把悬空与落地混成一种惯量。它执行：

1. 按反馈序号去重，按反馈接收时间重采样，剔除过时、饱和和变化率受限数据；
   差分窗口不跨角度/幅值/阶段切换或坏样本。
2. 使用实际准备指令的时间序列拟合主机延迟、iq 一阶响应、增益与偏置；独立阶跃段检查自由运行预测。
3. 使用实测 iq 和物理状态拟合等效角加速度：
   `q_dd = c0*i + c1*i*(q-q0) + c2*i*abs(v) + c3*v + c4*tanh(10*v) + c5*sin(q) + c6*cos(q) + c7`。
   检查双向探针/速度、角度跨度、矩阵秩/条件数、独立加速度预测与 0.5 s 位置/速度滚动预测。

NPZ 中训练/验证按阶段严格分开，列名随文件导出，标签是 `acceleration_rad_s2`，没有伪造的
`torque_Nm`。无法辨识或验证失败会写明原因，并以退出码 2 和 `training_ready: false` 阻止直接使用。
每个模型记录电流/角度/速度/温度/频率覆盖范围。闭环最小二乘仍可能有噪声偏差，验证通过也只支持
实际覆盖的工况；温度、负载、接触和机构耦合超出覆盖时须扩展实验和模型。

`.script/identification/current_actuator_model.py` 提供读取与计数响应模拟：

- `step_current(command_raw, dt)` 模拟量化、有效延迟及 iq 响应。
- `acceleration(physical_angle, physical_velocity, iq_raw)` 预测已覆盖状态的等效物理角加速度。
- `equivalent_simulation_torque(..., simulation_inertia, passive_simulation_torque)` 在显式提供
  该坐标下的等效惯量与被动/接触项后计算 `J_sim * q_dd - tau_passive_sim`。
  这是匹配加速度所需的**仿真等效扭矩**，不是测得的真实输出轴扭矩。闭链、多体耦合须使用对应的
  完整动力学映射，不能把一个电机裸转子的标称惯量当成整个机构惯量。

如果仿真使用 RL 缩放角而不是物理腿角，须先用车辆标定做位置、速度、加速度和广义力变换：
`q_rl = k*(alpha_high-alpha)`，`k=q_rl_max/(alpha_high-alpha_low)`；
`q_rl_dd = -k*alpha_dd`，广义力满足 `tau_rl = -tau_alpha/k`。
被动重力/接触项必须与该坐标一致，避免在拟合响应和仿真中重复加重力。

没有独立外力、惯量或机械力矩标定时，扫描运动可以辨识 `Kt/J` 一类等效关系，无法唯一拆出真实
`Kt` 与机构惯量，更无法仅凭“用手卡住”得到绝对 N·m。新流程让 real2sim 不依赖当前错误的 `/torque`
值；真实辨识参数和 RL 改善效果仍需用户实际采集与验证。

## 已有验证与限制

离线 C++ 测试检查 CAN 计数与方向、协议边界、反馈帧类别/时间戳、ADRC 两层旧限幅移除、
ESO 已编码输入、完整插件与四路 ADRC 的无环装配、显式开始/停止、双下复位和重新开始。
Python 合成系统测试覆盖指令延迟/一阶动态、重复反馈、饱和剔除、模型辨识、独立验证、
工况/覆盖范围拒绝、未完成/丢行拒绝及源配置一致性。这些都是软件和合成数据结果。

本次三个硬件源文件、ADRC、新扫频插件目标文件编译通过。完整 `rmcs_core` 构建在未改动的
`px4_vision_bridge.cpp` 因环境缺失 `mavlink/v2.0/common/mavlink.h` 中止，尚未完成完整链接/安装，
也没有真车数据或真实 RL 训练验证。
