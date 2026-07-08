# 变形底盘控制模块重构方案（Plan）

本文档给出 `deformable chassis` 相关控制模块的重构蓝图，目标不是改变控制行为，而是在尽量不改语义的前提下，整理模块边界、降低头文件复杂度、统一与仓库现有 C++ 风格的契合度。

本文档覆盖以下三个核心文件：

1. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_mode_manager.hpp`
2. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_joint_manager.hpp`
3. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_suspension.hpp`

必要时会同时提到与其协作的：

1. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`
2. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp`
3. `rmcs_ws/src/rmcs_core/src/controller/chassis/omni_wheel_controller.cpp`
4. `rmcs_ws/src/rmcs_core/src/controller/chassis/qcp_solver.hpp`
5. `rmcs_ws/src/rmcs_core/src/controller/gimbal/two_axis_gimbal_solver.hpp`
6. `rmcs_ws/src/rmcs_core/src/hardware/device/lk_motor.hpp`

## 1. 重构目标

本次重构的目标不是“把所有 `struct` 改成 `class`”，也不是“为了抽象而抽象”，而是：

1. 保留当前底盘控制行为与参数语义。
2. 让头文件回到“声明为主、实现为辅”的状态。
3. 让每个类的职责边界更清晰。
4. 让成员变量、辅助函数和嵌套类型的布局更符合仓库中更成熟的控制器/设备类写法。
5. 为后续继续拆分主动悬挂、被动悬挂、输入状态机等逻辑留出空间。

## 2. 当前问题判断

结合仓库中较稳定的样本文件，例如：

1. `controller/chassis/hero_chassis_controller.cpp`
2. `controller/chassis/omni_wheel_controller.cpp`
3. `controller/chassis/qcp_solver.hpp`
4. `controller/gimbal/two_axis_gimbal_solver.hpp`
5. `hardware/device/lk_motor.hpp`
6. `hardware/device/supercap.hpp`
7. `hardware/device/vt13.hpp`

可以得出如下判断：

1. 仓库允许在 `class` 中定义 `struct`。
2. 仓库中的 `struct` 主要承担四类角色：配置对象、结果对象、约束对象、协议数据对象。
3. 仓库中的大型组件通常把主调度逻辑放在 `.cpp`，而不是把所有非模板实现都塞在 `.hpp`。
4. 当前 `deformable` 相关头文件的主要问题不是 `struct` 过多，而是状态机、轨迹、标定、修正等实现过于集中在头文件中。

因此，本方案的主线是：

1. 保留合理的 `struct`。
2. 下沉重实现到 `.cpp`。
3. 重新整理 `public/private` 排布与成员分组。
4. 只做最小必要的命名调整，不做大规模概念替换。

## 3. 适用的仓库风格结论

从现有代码风格提炼出的可复用结论如下：

1. `class` 用于表示带行为、带生命周期、带封装边界的对象。
2. `struct` 用于表示轻量数据聚合，不等于“不是面向对象”。
3. 若一个嵌套类型只服务于当前类，继续放在类中通常比抽到命名空间级更合适。
4. 若一个函数不是模板、也不要求强内联，优先放入 `.cpp`。
5. 类的排布通常推荐为：常量与类型、构造与公有接口、私有辅助函数、成员变量。
6. 成员变量推荐按“输出状态/接口对象 -> 配置参数 -> 运行时状态 -> 上一帧输入历史”的顺序组织。

## 4. 文件范围与落地方式

建议新增以下实现文件：

1. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_mode_manager.cpp`
2. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_joint_manager.cpp`
3. `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis_suspension.cpp`

建议保留现有头文件文件名，不在第一轮重构中改名，以减少 include 级联影响。

## 5. 各文件中应保留哪些 struct

### 5.1 `deformable_chassis_mode_manager.hpp`

建议保留：

1. `enum class SuspensionMode`
2. `struct JointPostureState`

保留原因：

1. `SuspensionMode` 是清晰的局部状态类型。
2. `JointPostureState` 是标准的“状态输出聚合对象”，它描述的是 mode manager 的对外结果，而不是内部实现细节。
3. 当前 `deformable_chassis.cpp` 通过 `joint_posture_state()` 读取该结构体，语义非常自然。

不建议在第一轮重构中把 `JointPostureState` 抽到命名空间级，原因是：

1. 目前它主要由 `DeformableChassisModeManager` 生成。
2. 它的命名和语义仍然强依赖 mode manager。
3. 抽出后收益不高，但会扩大接口面。

### 5.2 `deformable_chassis_joint_manager.hpp`

建议第一轮不新增新的对外 `struct`。

说明：

1. 当前 `DeformableChassisJointManager` 的接口已经比较集中。
2. 它对外暴露的是按 joint 访问的 getter，而不是一个复杂聚合结果。
3. 如果未来需要进一步降复杂度，可以在第二轮考虑引入内部 `JointTrajectoryState`，但第一轮不建议为了抽象而新增类型。

### 5.3 `deformable_chassis_suspension.hpp`

建议保留：

1. `struct Corrections`

保留原因：

1. `Corrections` 是标准结果对象，与 `TwoAxisGimbalSolver::AngleError` 的定位一致。
2. 该结构体清楚表达了上层调度器最终需要的内容：角度修正、速度修正、加速度修正、每 joint 是否激活。
3. 改成 `class` 或拆成多个返回值都会增加调用复杂度。

### 5.4 明确不建议保留为 struct 的情况

以下内容若未来要新增，不建议默认用 `struct`：

1. 带复杂不变量维护的状态机对象。
2. 需要隐藏内部状态和 reset/update 规则的运行时对象。
3. 未来可能独立演化的 planner/controller 子模块。

这些更适合用 `class`。

## 6. 哪些函数应移到 `.cpp`

### 6.1 `deformable_chassis_mode_manager.hpp`

建议保留在头文件中的内容：

1. 小型 getter：`joint_posture_state()`、`min_angle()`、`max_angle()`、`max_angle_rad()`、`active_suspension_min_angle_rad()`、`correction_inverted()`
2. 极小静态工具函数若确认只用一处且足够短，可继续内联，例如 `deg_to_rad_()`

建议移入 `deformable_chassis_mode_manager.cpp` 的函数：

1. 构造函数 `DeformableChassisModeManager(rclcpp::Node& node)`
2. `reset()`
3. `update(...)`
4. `update_mode_from_inputs_(...)`
5. `activate_complex_spin_(...)`
6. `deactivate_complex_spin_()`
7. `apply_front_high_rear_low_target_()`
8. `apply_front_low_rear_high_target_()`
9. `toggle_bg_target_()`
10. `update_suspension_mode_from_inputs_(...)`
11. `update_remote_low_prone_toggle_from_inputs_(...)`
12. `remote_low_prone_toggle_requested_by_switch_(...)`
13. `update_lift_target_toggle_(...)`
14. `rotary_knob_down_edge_(...)`
15. `rotary_knob_up_edge_(...)`
16. `update_joint_posture_state_(...)`
17. `symmetric_joint_target_requested_(...)`

原因：

1. 这些都不是模板，也不依赖必须头文件可见的实现。
2. 它们共同构成复杂输入状态机，放在 `.cpp` 更利于按主流程阅读。
3. 改动其中任一逻辑时，不应迫使所有 include 该头文件的翻译单元重新编译。

### 6.2 `deformable_chassis_joint_manager.hpp`

建议保留在头文件中的内容：

1. `kJointCount`
2. 默认构造函数
3. 很短的 getter：`angle_state()`、`velocity_state()`、`acceleration_state()`、`joint_active()`、`angle_states()`
4. 若保守起见，`deg_to_rad_()` 也可保留在头文件

建议移入 `deformable_chassis_joint_manager.cpp` 的函数：

1. `configure(rclcpp::Node& node)`
2. `reset()`
3. `init_from_feedback(...)`
4. `any_active() const`
5. `run_trajectory(...)`

原因：

1. 这些函数虽然职责单一，但实现并不短。
2. `run_trajectory(...)` 是实打实的轨迹推进逻辑，放 `.cpp` 后更适合作为单独算法块阅读。

### 6.3 `deformable_chassis_suspension.hpp`

建议保留在头文件中的内容：

1. `Corrections`
2. 很短的状态 getter：`calibrated()`、`pitch_offset()`、`roll_offset()`、`scope_torque()`
3. `deg_to_rad_()` 若不想暴露额外符号也可继续内联

建议移入 `deformable_chassis_suspension.cpp` 的函数：

1. `configure(rclcpp::Node& node)`
2. `calibrate(...)`
3. `update(...)`
4. `reset()`
5. `load_pid_(...)`
6. `reset_attitude_()`
7. `reset_passive_state_()`
8. `reset_calibration_window_()`
9. `compute_correction_targets_(...)`
10. `update_passive_targets_(...)`
11. `run_correction_trajectory_(...)`

原因：

1. `configure(...)` 和 `update(...)` 已经承担多个控制子阶段。
2. 主动悬挂、被动悬挂、校准流程、修正轨迹不应在头文件内平铺。
3. `.cpp` 更便于后续按阶段重排函数顺序。

## 7. `public/private` 推荐最终排布

### 7.1 `DeformableChassisModeManager`

推荐结构：

1. `public` 嵌套类型
2. `public` 构造、`reset()`、`update()`、状态 getter
3. `private` 小工具函数
4. `private` 输入状态机更新函数
5. `private` posture target 更新函数
6. `private` 成员变量

推荐排布示意：

```cpp
class DeformableChassisModeManager {
public:
    enum class SuspensionMode : uint8_t { ... };

    struct JointPostureState { ... };

    explicit DeformableChassisModeManager(rclcpp::Node& node);

    void reset();
    void update(...);

    const JointPostureState& joint_posture_state() const;
    double min_angle() const;
    double max_angle() const;
    double max_angle_rad() const;
    double active_suspension_min_angle_rad() const;
    bool correction_inverted() const;

private:
    static double deg_to_rad_(double deg);
    static bool symmetric_joint_target_requested_(...);

    void update_mode_from_inputs_(...);
    void update_suspension_mode_from_inputs_(...);
    void update_remote_low_prone_toggle_from_inputs_(...);
    bool remote_low_prone_toggle_requested_by_switch_(...);

    void activate_complex_spin_(...);
    void deactivate_complex_spin_();
    void apply_front_high_rear_low_target_();
    void apply_front_low_rear_high_target_();
    void toggle_bg_target_();
    void update_lift_target_toggle_(...);
    bool rotary_knob_down_edge_(...) const;
    bool rotary_knob_up_edge_(...) const;
    void update_joint_posture_state_(bool low_prone_active);

    JointPostureState joint_posture_state_;

    double min_angle_;
    double max_angle_;
    bool suspension_enable_;
    bool passive_suspension_enable_;

    double current_target_angle_;
    std::array<double, kJointCount> joint_current_target_angle_;

    bool apply_symmetric_target_;
    bool complex_spin_active_;
    double complex_spin_elapsed_;
    bool suspension_enabled_by_toggle_;
    bool passive_suspension_enabled_by_toggle_;
    bool remote_low_prone_enabled_by_toggle_;
    bool remote_low_prone_toggle_left_down_pending_;

    rmcs_msgs::Switch last_switch_right_;
    rmcs_msgs::Switch last_switch_left_;
    rmcs_msgs::Keyboard last_keyboard_;
    double last_rotary_knob_;
};
```

核心原则：

1. 先让读者看见“这个类对外输出什么”。
2. 再看“它有哪些输入状态机步骤”。
3. 最后看“内部到底记住了哪些跨帧状态”。

### 7.2 `DeformableChassisJointManager`

推荐结构：

1. `public` 常量和构造
2. `public` 生命周期接口：`configure()`、`reset()`
3. `public` 运行接口：`init_from_feedback()`、`run_trajectory()`
4. `public` 查询接口：`any_active()`、各 getter
5. `private` 小工具与成员

推荐排布示意：

```cpp
class DeformableChassisJointManager {
public:
    static constexpr size_t kJointCount = 4;

    DeformableChassisJointManager() = default;

    void configure(rclcpp::Node& node);
    void reset();

    bool init_from_feedback(const std::array<double, kJointCount>& physical_angles);
    void run_trajectory(
        const std::array<double, kJointCount>& target_angles_rad,
        bool suspension_active,
        double dt);

    bool any_active() const;
    double angle_state(size_t i) const;
    double velocity_state(size_t i) const;
    double acceleration_state(size_t i) const;
    bool joint_active(size_t i) const;
    std::array<double, kJointCount> angle_states() const;

private:
    static double deg_to_rad_(double deg);

    std::array<bool, kJointCount> joint_target_active_;
    std::array<double, kJointCount> target_angle_state_rad_;
    std::array<double, kJointCount> target_velocity_state_rad_;
    std::array<double, kJointCount> target_acceleration_state_rad_;

    double target_vel_limit_;
    double target_acc_limit_;
    double suspension_vel_limit_;
    double suspension_acc_limit_;
};
```

这里不建议为追求形式对称而新增 `private/public` 过多层级，保持紧凑即可。

### 7.3 `DeformableChassisActiveSuspension`

推荐结构：

1. `public` 结果类型 `Corrections`
2. `public` 生命周期接口：`configure()`、`reset()`
3. `public` 标定与主更新接口：`calibrate()`、`update()`
4. `public` 简短状态 getter
5. `private` 配置加载与状态复位 helper
6. `private` 主动悬挂相关 helper
7. `private` 被动悬挂相关 helper
8. `private` correction trajectory helper
9. `private` 成员变量按语义分组

推荐成员变量分组顺序：

1. 常量与 joint index
2. PID 控制器对象
3. 主动悬挂配置参数
4. 被动悬挂配置参数
5. 标定配置与标定运行时状态
6. 被动载荷滤波与积分状态
7. correction target/state/velocity/acceleration

推荐排布示意：

```cpp
class DeformableChassisActiveSuspension {
public:
    struct Corrections { ... };

    void configure(rclcpp::Node& node);
    void calibrate(...);
    Corrections update(...);
    void reset();

    bool calibrated() const;
    double pitch_offset() const;
    double roll_offset() const;
    double scope_torque(bool suspension_active, bool is_spin) const;

private:
    static double deg_to_rad_(double deg);
    void load_pid_(...);
    void reset_attitude_();
    void reset_passive_state_();
    void reset_calibration_window_();

    void compute_correction_targets_(...);
    void update_passive_targets_(...);
    void run_correction_trajectory_(...);

    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};

    double active_correction_vel_limit_;
    double active_correction_acc_limit_;

    bool passive_enabled_;
    double passive_kp_;
    double passive_ki_;
    double passive_deadband_;
    double passive_integral_limit_;
    double passive_load_lpf_alpha_;
    double passive_max_correction_rad_;
    double passive_correction_vel_limit_;
    double passive_correction_acc_limit_;
    std::array<double, kJointCount> passive_load_sign_;
    std::array<double, kJointCount> passive_load_bias_;

    double calibration_wait_time_;
    double calibration_sample_time_;
    double calibration_hold_elapsed_;
    size_t sample_count_;
    double pitch_sum_;
    double roll_sum_;
    bool calibration_completed_for_window_;
    bool calibrated_once_;
    double pitch_offset_;
    double roll_offset_;

    bool passive_load_initialized_;
    std::array<double, kJointCount> passive_filtered_load_proxy_;
    std::array<double, kJointCount> passive_load_integral_;

    std::array<double, kJointCount> correction_target_rad_;
    std::array<double, kJointCount> correction_state_rad_;
    std::array<double, kJointCount> correction_velocity_state_rad_;
    std::array<double, kJointCount> correction_acceleration_state_rad_;
};
```

## 8. 成员变量怎么放，为什么这样放

这部分给出更具体的分组准则，便于真正落地时保持统一。

### 8.1 先放“对外结果”，再放“配置”，最后放“运行时痕迹”

原因：

1. 对阅读者来说，先知道类输出什么最重要。
2. 配置参数是稳定背景信息。
3. 运行时状态和上一帧输入属于实现细节，应放得更靠后。

### 8.2 跨帧边沿检测状态应单独成组

例如：

1. `last_switch_right_`
2. `last_switch_left_`
3. `last_keyboard_`
4. `last_rotary_knob_`

这些变量的共同特征是：

1. 它们不代表业务输出。
2. 它们只服务于 edge detect / toggle detect。
3. 单独成组后，读者不会误以为它们是控制目标或配置参数。

### 8.3 “当前目标”和“修正状态”不能混排

例如在 `ModeManager` 中：

1. `current_target_angle_`
2. `joint_current_target_angle_`

应和：

1. `complex_spin_active_`
2. `remote_low_prone_enabled_by_toggle_`

分开，因为前者是“控制目标状态”，后者是“输入状态机记忆”。

### 8.4 标定状态应与控制状态分离

在 `ActiveSuspension` 中，以下变量应放在一起：

1. `calibration_wait_time_`
2. `calibration_sample_time_`
3. `calibration_hold_elapsed_`
4. `sample_count_`
5. `pitch_sum_`
6. `roll_sum_`
7. `calibration_completed_for_window_`
8. `calibrated_once_`
9. `pitch_offset_`
10. `roll_offset_`

这样能明显区分：

1. 哪些变量属于上电/模式切换后的标定窗口。
2. 哪些变量属于每帧控制输出。

## 9. 哪些函数名建议调整，哪些不该动

### 9.1 建议保守处理命名

第一轮重构的主目标是“整理结构，不改语义”，因此命名调整应克制。建议默认：

1. 能不改就不改。
2. 只改明显误导或不够自解释的名字。
3. 避免一次性改动太多函数名，导致 review 成本升高。

### 9.2 `DeformableChassisModeManager` 中建议调整的命名

建议改：

1. `update_lift_target_toggle_()` -> `update_posture_target_from_inputs_()`

原因：

1. 这个函数实际不只做 toggle。
2. 它同时处理对称姿态切换、前高后低、前低后高、复杂 spin 定时切换。
3. “lift target toggle” 的覆盖面过窄，不能准确表达函数职责。

可选改名，若想进一步增强可读性：

1. `toggle_bg_target_()` -> `toggle_front_back_posture_target_()`
2. `apply_front_high_rear_low_target_()` 保持不变
3. `apply_front_low_rear_high_target_()` 保持不变

其中：

1. `apply_front_high_rear_low_target_()` 与 `apply_front_low_rear_high_target_()` 已经足够清楚，不建议再动。
2. `toggle_bg_target_()` 中的 `bg` 可读性较弱，值得改名。

### 9.3 `DeformableChassisJointManager` 中建议不改的命名

以下名称建议保持：

1. `configure()`
2. `reset()`
3. `init_from_feedback()`
4. `run_trajectory()`
5. `any_active()`
6. `angle_state()`
7. `velocity_state()`
8. `acceleration_state()`

原因：

1. 这些名称已经简洁且符合职责。
2. 进一步改名收益很低。

### 9.4 `DeformableChassisActiveSuspension` 中建议保留的命名

建议保持：

1. `configure()`
2. `calibrate()`
3. `update()`
4. `reset()`
5. `compute_correction_targets_()`
6. `update_passive_targets_()`
7. `run_correction_trajectory_()`

原因：

1. 这些名字与其功能基本一致。
2. 真正的问题在于实现混在头文件，而不是名字不对。

### 9.5 不建议在第一轮做的命名调整

不建议做：

1. 把所有成员名里的 `_state_`、`_target_`、`_active_` 统一重命名。
2. 把 `joint_posture_state_` 改成别的聚合概念名。
3. 把 `DeformableChassisActiveSuspension` 改名成更大的架构名。

原因：

1. 这类改动会提高 diff 噪音。
2. 当前首要问题不是术语统一，而是结构可维护性。

## 10. 分文件重构步骤建议

### 10.1 第一阶段：只做接口/实现分离

顺序建议：

1. `deformable_chassis_mode_manager.hpp/.cpp`
2. `deformable_chassis_joint_manager.hpp/.cpp`
3. `deformable_chassis_suspension.hpp/.cpp`

阶段目标：

1. 保证行为不变。
2. 保证 include 不变。
3. 保证 `deformable_chassis.cpp` 调用层几乎无需修改。

### 10.2 第二阶段：只做排布整理

内容：

1. 重排 `public/private` 区块。
2. 重排成员变量分组。
3. 在 `.cpp` 中按主流程顺序排布函数。

阶段目标：

1. 让类定义本身可读。
2. 让 `.cpp` 从上往下阅读时呈现“主流程 -> 细节 helper”的结构。

### 10.3 第三阶段：最小命名修正

只建议处理：

1. `update_lift_target_toggle_()`
2. `toggle_bg_target_()`

其余命名先不动。

### 10.4 第四阶段：再评估是否值得继续拆类

完成前三阶段后，再判断是否需要更进一步的结构拆分，例如：

1. 把 mode manager 中的 remote input edge detect 抽成更小 helper。
2. 把 active/passive suspension 逻辑拆成两个对象。
3. 把 correction trajectory 的推进逻辑提取成共享内部算法。

第一轮不建议直接做这些，因为风险比收益更大。

## 11. 不在本轮范围内的事项

以下内容明确不在本轮重构范围：

1. 修改底盘行为、切换条件或控制参数默认值。
2. 新增新的控制模式。
3. 改写主动/被动悬挂算法本身。
4. 对 `deformable_joint_controller.cpp` 的 ADRC 接口做扩展。
5. 对 `deformable_chassis.cpp` 主调度流程做架构性改写。

## 12. 验收标准

本轮重构完成后，应满足以下验收条件：

1. 三个头文件的主体从“实现承载体”变成“接口说明”。
2. 合理的 `struct` 被保留，而不是被无意义替换成 `class`。
3. 新增 `.cpp` 后，`deformable_chassis.cpp` 的调用方式基本不变。
4. 成员变量分组能够让读者明显区分：配置、输出状态、运行时状态、输入历史。
5. 仅做有限命名调整，不引入大面积语义改写。

## 13. 结论

这批 `deformable` 文件当前最主要的问题，不是“用了 `struct` 所以不面向对象”，而是：

1. 非模板控制逻辑过度堆积在头文件里。
2. 一个类内部承载了过多实现细节，但排布层次不清晰。
3. 对外接口与内部状态机/轨迹逻辑没有被很好地分层展示。

因此，推荐的重构路线是：

1. 保留 `JointPostureState` 和 `Corrections` 这类合理的结果型 `struct`。
2. 将重逻辑迁移到各自 `.cpp`。
3. 统一 `public/private` 与成员变量分组方式。
4. 只对少数误导性命名做最小修正。

如果按本 Plan 实施，第一轮重构应优先追求“结构更清楚但行为不变”，而不是一次性把控制代码拆成过度细碎的新抽象。
