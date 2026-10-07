# deformable 四腿同步角度扫频与 real2sim 数据

## 实验流程

悬空、落地均四条腿同步运动，使用车辆原 `DeformableJointController` ADRC 和参数。
不再固定角度叠加电流探针，不再逐腿扫。DR16 左右双中开始一次实验：

1. 从当前角度以 0.08 rad/s 将参考移动到 min_angle。
2. 慢扫 120 秒，角度 chirp 频率从 0.02 Hz 到 0.06 Hz。
3. 返回 min_angle，再快扫 60 秒，频率从 0.06 Hz 到 0.12 Hz。
4. 返回 min_angle，增加极快段：1 Hz，即每 1 秒 min→max→min，连续 10 秒。
5. 仅悬空工况再增加阶跃段：目标在 min/max 间做方波阶跃，每级保持 5 秒，共 3 个周期，
   用于观察当前 ADRC 参数下电机的最大输出与饱和情况；落地工况跳过该段。
6. 返回 min_angle，完成、输出归零、CSV 关盘。

极快段使用相同余弦角度轨迹和解析速度，仅提高运动频率，四腿仍同步；
`ultra_fast_frequency_hz: 1.0`、`ultra_fast_duration_s: 10.0` 可在两份 YAML 调整。
这是目标往返周期，实际角度/速度能否跟上应根据记录判断。
17–75°范围下目标峰值速度约 3.18 rad/s，峰值加速度约 19.98 rad/s²。
可比较最终指令 torque 代理与反馈 torque 代理的时序来估计有效延迟；
角度参考到实测角度的滞后还包含 ADRC 和机械响应，不能直接当成电机出力延迟。

阶跃段只在悬空 YAML 出现（`step_enable: true`、`step_duration_s: 5.0`、`step_cycles: 3`），
阶段号 10，四腿目标方波同相同步。阶跃记录里：
`target_torque_proxy` 是 ADRC 未限幅请求，`command_current_raw`/`command_torque_proxy`
是最终编码指令，`command_limited` 表示被 ±2048 协议上限截断。
`target_torque_proxy` 达到 `2048 × torque_per_current_raw` 且 `command_limited=1`
即为当前参数下电机可输出的最大值（MG5010Ei36 代理系数 0.1740234375，上限约 356.4 代理单位）。
汇总脚本对 step 段单独输出每关节的饱和占比、峰值电流和峰值 torque 代理。

两遍参考角度都覆盖源配置的 min_angle–max_angle（当前 17–75°）：
`q_ref = min + (max-min)/2 * (1-cos(phase))`。
速度参考是这条曲线的解析导数。四路目标完全同相，实际跟踪由各自 ADRC 决定。
初始化从各腿当前位置平滑定位，因此定位阶段各腿参考可能不同，chirp 阶段相同。
轨迹范围约束的是参考，不是实际测量角度。按用户要求没有角度/温度/超时等自动中止。
离开双中、双下或手动 stop 可停止。完成后保持双中不重启，重新双中才启动下一次。

阶段：idle=0，初始定位=1，慢扫后返回=2，慢扫=3，快扫后返回=4，完成=5，停止=6，
快扫=7，极快扫=8，最终返回=9，阶跃响应=10（仅悬空）。

## 编译、同步、自动取回

在开发 Docker 中运行：

```bash
build-rmcs --packages-select rmcs_core rmcs_bringup
sync-remote
```

`sync-remote` 同步安装产物。同步完成后重启车上 RMCS。
选用 `deformable-current-sweep-suspended` 或 `deformable-current-sweep-grounded`。
DR16 双中后自动采集，完成后由用户 SCP 到电脑。例如：

```bash
scp remote:/tmp/deformable_angle_sweep_suspended_实际时间_run1.csv* ./data/identification/
```

如果需要可选的自动取回，可以单独运行：

```bash
python3 .script/identification/collect_current_sweep.py --condition all --watch --raw-only
```

文件保存到电脑绑定挂载的 `RMCS/data/identification/`，机器人原文件保留。
后台关盘后原子生成 `.csv.status.json`，取回程序等待它出现，再 SCP CSV、元数据与状态。
不会复制正在写的半个文件。每个 run 一个文件，`.received` 标记已取回。
角度扫频自动生成 `.summary.json` 统计慢/快两遍的角度、速度、torque 与饱和范围。

状态查询：

```bash
ros2 service call /identification/status std_srvs/srv/Trigger '{}'
ros2 service call /identification/collector_status std_srvs/srv/Trigger '{}'
ros2 service call /identification/stop std_srvs/srv/Trigger '{}'
```

## ValueCollector 与 CSV 数据

两份配置启用 `rmcs_core::debug::ValueCollector -> value_collector`，参数为：

```yaml
experiment_mode: true
csv_path: /tmp/deformable_angle_sweep_<condition>_<timestamp>.csv
write_interval: 5
flush_interval: 1000
```

1000 Hz 控制、每 5 周期一行，约 200 Hz CSV；每 1000 行 flush，结束必定关盘。
实验模式用后台写线程，保持 ValueCollector 的 signals 接口。写失败/丢行在 status 中记录。

每个 `/chassis/<joint>_joint` 的列：

| 列后缀 | 定义 |
|---|---|
| physical_angle / physical_velocity | 物理腿角 rad、rad/s |
| rl_angle / rl_velocity | 车辆标定的 RL 缩放坐标 |
| target_physical_angle / target_physical_velocity | 同步轨迹目标角度、解析目标速度 |
| torque | 原驱动电流换算 torque 代理，电机配置坐标，不是真实测力 |
| torque_per_current_raw | 电机配置坐标下每个方向修正电流计数的代理 Nm 系数 |
| target_torque_proxy | ADRC 请求电流 × 上述系数，尚未按 CAN 协议限幅/取整 |
| command_torque_proxy | 最终编码 CAN 电流 × 上述系数，与本行 command_current_raw 对齐 |
| physical_torque_proxy | `-torque`，当前物理腿角与电机坐标方向相反 |
| target_physical_torque_proxy / command_physical_torque_proxy | 上述请求/最终 torque 代理取负，物理腿角坐标 |
| current_raw / raw_current | 配置方向电流计数 / CAN 原始 iq 计数 |
| servo_current_raw / control_current_raw | ADRC 请求 / 送给硬件的请求计数（无探针） |
| command_current_raw / command_limited | 最终准备发送的 CAN 计数 / 编码限幅标志 |
| temperature | 摄氏温度 |
| feedback_sequence / feedback_time_s / feedback_age_s | 反馈序号、主机接收单调时间、反馈年龄 |
| command_sequence / command_prepare_time_s | 指令序号、主机准备 CAN 包时间 |

MG5010Ei36 当前代理系数：`33/2048 * 0.3 * 36 = 0.1740234375` Nm/count。
这些 Nm 都是厂家常数定义的估算单位，既不是校准结果，也不是输出轴实测力矩。

公共列为 host_time_s、elapsed_s、run、stage、frequency_hz、actuation_active、DR16 左右状态，
以及机体 roll/pitch。host_time_s 在硬件命令准备后采集，浮点精度 17 位。

## RL real2sim 使用

原始 CSV 提供“目标角/速度 → ADRC 目标 torque → 最终指令 → 反馈 torque → 实际角/速度”链。
可辨识有效指令延迟、响应时间、电流/torque 代理跟踪、摩擦/阻尼、姿态负载和快慢轨迹迟滞。
仿真使用同一角度轨迹与 ADRC，优化角度、速度和代理 torque 的差异，参数按工况分别拟合。
同步四腿数据是耦合整机响应，不能解释成四个独立单关节模型。

准备时间不是总线实际发送时间、接收时间不是电机内部采样时间；推得的是整条链的有效延迟。
反馈应按序号去重。饱和、丢行和过时数据需离线标注；跑完不等于参数已识别。
旧 `fit_current_sweep.py`、静载表脚本针对旧电流探针 schema 1，不能用于新的角度 schema 2。
连续角度扫频没有静止保持网格，若要静态 torque 表，应另外采集静态保持数据。

硬件实验禁用超级电容，不要求裁判状态/充电功率接口。A/B/C USB 对象先销毁并等待回调退出，
再销毁电机、IMU、监控和遥控器，修复启动失败时的退出段错误。
