# 轮腿执行器与部署延迟辨识

## 测量边界

`wheel-leg-latency.yaml` 是独立台架实验，不能与 RL 控制器同时加载。`WheelLegLatencyProbeController` 是唯一六轴力矩生产者；硬件仍是唯一 CAN 写入者。轮端减速比沿用 15.62，测试限幅不超过 ±4.5 N·m。髋膝仍由主机保持位置，DM MIT 帧的电机端 Kp、Kd 为零。

可直接测量主机控制计算、命令批次组装/SDK flush 返回、策略工作线程排队、推理、结果准备、首次发布、目标接收、首次 PD 使用的时间。所有时间均为主机 `steady_clock` 的整数纳秒；先相减再转浮点。主机组装时间、SDK 返回时间不代表 USB 完成、板端 CAN 发出或电机接收。

命令到电流反馈只能拟合**主机帧组装至上报电流等效力矩的有效延迟与一阶时间常数**。它包含下行、驱动响应、反馈采样/滤波、上行。CAN 反馈序号独立计数，不是命令的确认序号。摩擦导致的首次运动时间不能当作 FOC 延迟。

纯 FOC 延迟、USB/板端/CAN 单向通信、反馈采样滤波延迟保留 `null`。分离这些量需要板端收包/CAN 发送时间戳，以及同步电流探头或驱动内部 iq 参考锁存、ADC/PWM 时间戳。现有 librmcs v3.3.0b1 的普通 CAN 回调不包含这些板端时间，不能将 RTT 除二当作实测单向延迟。

## 台架采集

在实际运行 RMCS 的环境中准备配置。容器中使用 `/workspaces/RMCS`，主机直接运行时使用主机绝对路径。准备命令只写文件，不启动硬件。

```bash
python3 .script/identification/prepare_actuator_latency.py \
  --axis left_wheel \
  --output /workspaces/RMCS/.codex/report/latency-left-wheel-001
```

六个轴名为 `left_wheel`、`right_wheel`、`left_hip_joint`、`left_knee_joint`、`right_hip_joint`、`right_knee_joint`。每个实验使用新目录。准备器冻结完整参数、源文件、当前可用二进制和提交身份，并生成 `launch-command.txt`。事件文件使用排他创建，防止重跑覆盖录包。启动前核对当前构建与冻结文件一致。

轮子悬空、车体固定；每次只激励选定轮。先在零速测量，再通过 `--wheel-velocity 5` 或 `--wheel-velocity -5` 选择明确的速度工况。腿由台架支撑，气簧保持安装，先由操作者置于目标姿态；程序捕获当前同侧两个电机位置作为保持目标，不自动执行转腿或变高轨迹。建议分别在 50°、75°、100° 内角附近重复，并根据距机械边界的余量选择 `--joint-excursion`。该参数限制电机轴相对起点位移，不能替代机构内角标定。

```bash
python3 .script/identification/prepare_actuator_latency.py \
  --axis right_knee_joint --amplitude 1.0 --joint-excursion 0.1 \
  --output /workspaces/RMCS/.codex/report/latency-right-knee-mid-001
```

启动生成的命令后，MIDDLE/MIDDLE 开始；其他开关状态输出零力矩；DOWN/DOWN 失能并清空保持积分和故障。完成状态锁存，DOWN/DOWN 后可再次开始。遥控丢失或反馈超时停止输出。实际位移、速度越界或已就绪驱动掉线会停止并记录故障，需要 DOWN/DOWN 复位。不要将这一入口用于自由接地平衡试验。

驱动就绪后先保持 2 s。腿部在此期间使用有界负载积分消除部分稳态误差，随后冻结积分，继续位置 PD 保持。探针叠加在主机保持力矩后，再统一限幅。默认 6 个段，每段 10 s，段间留 1 s；共约 68 s。前三段为 50%、75%、100% 幅值，后三段为对应幅值的独立验证信号。每段先使用正负 5/10/20/50 ms 脉冲，之后使用确定性伪随机信号；跳过调度 tick 不会改变后续信号的索引。`--bit-duration`、`--segment-duration`、`--amplitude` 控制信号，不需要改源码。

轮端默认探针峰值 0.3 N·m，关节默认 1 N·m。幅值是否足够取决于实际电流噪声和量化；不能因位置没动就继续增大到起动摩擦。饱和段整体排除，保留原始记录。温度、姿态、供电、固定方式以及轮的实际速度应随实验记录，不能将不同工况的结果静默合并。

## 分析

正常退出采集程序以排空后台事件队列。事件 1 为主机命令帧、2 为每一帧原始反馈到达、3 为主机命令批次、4 为活动探针的控制阶段、5 为该命令批次使用的 action 序号及 PD 起止时刻，9 为正常关闭及丢失计数。反馈电流直接通过与运行反馈相同的 codec 换算，避免 1 kHz `ValueCollector` 只保留最新反馈而漏帧。生产者不执行磁盘 I/O；队列溢出显式计数。强制杀死、磁盘写入失败、缺少关闭尾标或丢帧的录包不能通过辨识验收。

在安装 NumPy、SciPy、PyYAML 的分析环境执行：

```bash
python .script/identification/analyze_actuator_latency.py \
  .codex/report/latency-left-wheel-001/events.csv \
  --control .codex/report/latency-left-wheel-001/control.csv \
  --output .codex/report/latency-left-wheel-001/result.json
```

分析器按实际非等间隔命令时刻精确积分零阶保持的一阶响应，同时拟合延迟、时间常数、增益、偏置；不将延迟四舍五入到 2 ms，也不以运动响应代替电流响应。完整的前三段用于拟合，后三段仅做独立验证；不随机切分相邻 CSV 行。报告逐段误差、采样间隔分布、限定频段的频响/相干度和条件自举区间。时间戳本身的误差、未知反馈滤波与模型误差不包含在该自举区间中。

初始拟合判据为验证 NRMSE ≤ 0.2、每个验证段 NRMSE ≤ 0.3、正增益，以及参数未碰搜索上界且延迟/时间常数可区分。不能据此声称覆盖整个速度、负载或温度域。一阶模型失败时保留失败结果，不能用一个固定 delay 强行解释高阶、速率限制或迟滞。

## 部署链路

对现有完整 RL YAML 添加采集字段，不新增第二个力矩控制器：

```bash
python3 .script/identification/prepare_actuator_latency.py \
  --deployment rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-wheelbipe.yaml \
  --output /workspaces/RMCS/.codex/report/latency-deployment-001
```

它沿用原模型、观测、策略频率、200 Hz PD 和使能逻辑；准备文件不等于部署或硬件运行。`--control` 分析会按 action sequence 匹配观测帧就绪、排队、推理起止、结果准备、首次发布、目标接收、首次 PD 使用。保持同一动作的多次 tick 不重复计入首次延迟。未接收提交、待推理观测被覆盖、结果被覆盖、PD 前目标被替代分别计数。每个源反馈的采样/上行延迟依旧未知；`frame_source_oldest_ns` 只表示主机已收到的源样本年龄。

`wheel_leg_latency_bench MODEL.onnx OUTPUT.csv SECONDS` 是独立的本机 ONNX 工作线程测试，按 50 Hz 提交、1 kHz 读取，不构造硬件或 PD。它可验证时钟顺序、异步拷贝及工作线程延迟，但不能代表车载部署延迟或实际电机回路。

## 回填仿真

结果默认 `runtime_admissible=false`，待通信/测量归因和独立回放验证后再分配到仿真。先按源样本到观测、策略部署、目标保持、200 Hz PD、下行、驱动响应的因果顺序建模。不能同时把整个“命令到上报电流”有效延迟施加到真实力矩，并再次添加其中的反馈延迟。被动气簧和机械摩擦不经过通信延迟。小于 5 ms 的延迟不能在 200 Hz 物理步中简单向上取整；应使用分数延迟/事件子步或显式细步对照，并保留既有检查点频率合同。
