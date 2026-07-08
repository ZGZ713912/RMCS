# RMCS C++ Specification

本文档是 RMCS 仓库内 AI 与自动化 Agent 编写 C++、组件配置和相关文档时必须遵守的统一规范。

适用范围：

- `rmcs_ws/src/**` 下的 `.cpp`、`.hpp`、`.h`
- 与这些 C++ 代码直接耦合的 `plugins.xml`、`*.yaml`、`launch`、说明文档
- RMCS 组件、设备封装、裁判系统 UI、控制器、硬件适配相关改动

如果局部目录已经存在更强的子规范，例如 `rmcs_auto_aim_v2/AGENTS.md`，则必须在遵守本文档的同时，优先遵守该子目录规范。

**1. 总原则**

- 先观察当前目录和当前模块的现有写法，再修改代码。
- 优先做最小正确修改，不做无关重构。
- 不为了“统一风格”重写旧代码。
- 不主动引入额外抽象层、包装层、兼容层。
- 若同仓库内存在多套局部风格，优先延续当前子目录风格。

**2. RMCS 架构认识**

- RMCS 主体组件通过 `rmcs_executor::Component` 连接，而不是默认直接通过 ROS publisher/subscription 互连。
- 大部分所谓“topic 名”在 RMCS 中是 executor 内部接口名，通过 `register_input` / `register_output` 连接。
- 真实 ROS topic 广播通常只在 `broadcaster` 或明确需要 ROS API 的地方出现。
- 新控制逻辑默认先考虑做成 `rmcs_executor::Component`，而不是写成私有 detail 模块或直接 ROS pub/sub。

**3. 组件边界规则**

- 一个组件只负责一层职责。
- 底盘模式与速度意图属于 chassis controller。
- 关节目标轨迹、主动悬挂、被动悬挂、IMU 安装误差校准这类逻辑，应独立成悬挂组件，而不是藏在 chassis controller 的 detail 文件里。
- 若一个 `.cpp` 文件中的逻辑已经具备独立输入、独立状态、独立输出，就应优先考虑拆成单独组件。
- 禁止通过 `detail namespace + handle + free functions` 的方式把本应是组件的逻辑隐藏到另一个源文件里。

**4. 组件文件布局**

- RMCS 组件必须优先按 `SteeringWheelController` 这一类现有主流格式编写。
- 这不是建议，而是强约束：新增或重构 RMCS 组件时，默认只能按这种布局组织。
- 单组件 `.cpp` 文件必须优先保持以下结构：
  - include 区
  - `namespace ... {`
  - `class Xxx : public rmcs_executor::Component [, public rclcpp::Node]`
  - class 开头允许少量 `using` 或内部类型别名
  - `public:`
  - 构造函数
  - `before_pairing()` / `before_updating()` / `update()` 等生命周期函数
  - `private:`
  - 私有辅助结构体
  - 私有辅助函数
  - 接口成员
  - 算法对象、配置、状态成员
  - `} // namespace ...`
  - `#include <pluginlib/class_list_macros.hpp>`
  - `PLUGINLIB_EXPORT_CLASS(...)`
- 组件内禁止把 `private:` 放在前面，再把构造函数和 `update()` 放到后面。
- 组件内禁止先写一大段成员变量，再写构造函数。
- 组件内禁止随意改成别的类布局体系。
- `register_input(...)` 和 `register_output(...)` 必须直接写在组件构造函数体中。
- 禁止把接口注册封装进 `register_interfaces()`、`register_interfaces_()`、`init_interfaces()` 这类 helper。
- 如果组件只在一个 `.cpp` 内使用，不必强行拆头文件。
- 模板类、轻量工具类、设备封装、widget 类可以放 `.hpp`。

标准模板：

```cpp
namespace rmcs_core::controller::chassis {

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/example/input", input_);
        register_output("/example/output", output_);
    }

    void before_updating() override {
    }

    void update() override {
    }

private:
    struct ExampleState {
    };

    void reset_all_controls() {
    }

    InputInterface<double> input_;
    OutputInterface<double> output_;
    ExampleState state_;
};

} // namespace rmcs_core::controller::chassis
```

**5. 输入输出接口规则**

- 组件之间默认使用：
  - `register_input("/path", input_)`
  - `register_output("/path", output_)`
- 可选输入必须显式传 `false`，并在使用前检查 `.ready()`。
- 接口名必须稳定、直白、领域语义明确。
- 同一份数据如果已在系统中有既有接口名，不得随意改名。
- 新组件接入时，必须显式梳理：
  - 需要哪些输入
  - 提供哪些输出
  - 哪些输出由别的组件消费

**6. 插件和配置链路规则**

- 每个可加载组件都必须：
  - 在源码末尾 `PLUGINLIB_EXPORT_CLASS(..., rmcs_executor::Component)`
  - 在所在包的 `plugins.xml` 中注册
  - 在需要启动它的 YAML `components:` 列表中显式加入
- YAML 中组件名保持 `Class -> instance_name` 格式。
- 新增组件时，必须同时检查四处是否一致：
  - C++ 类全名
  - `PLUGINLIB_EXPORT_CLASS`
  - `plugins.xml`
  - `config/*.yaml`

**7. 参数与 YAML 规则**

- 组件读取的参数必须真实存在于对应实例名的 YAML 参数块中。
- YAML 里不存在但代码读取的参数，是错误。
- YAML 里存在但代码完全不读取、也没有文档说明未来用途的参数，视为死配置，应清理。
- 组件职责拆分后，参数也必须跟着职责迁移。
- 只有组件 A 读取的参数，必须放到组件 A 的参数块下，而不是历史上顺手留在别的组件块里。
- 不保留“看起来像新算法配置，但当前完全没有接到代码”的死参数。

**8. Reset 语义规范**

- DR16 双下是全系统显式 reset 信号。
- 进入 reset 时，组件必须：
  - 停止输出有效控制量
  - 清空内部状态
  - 清空积分器、滤波器、轨迹状态、校准窗口等持久状态
  - 恢复到明确的 reset 后初始状态
- 不能只把输出置零而保留旧内部状态。
- 若一个子逻辑已拆成独立组件，而 reset 触发来自另一个组件，必须通过显式接口把 reset 信号传过去，不能依赖隐式副作用。

**9. 文件命名规则**

- 新增源文件默认使用 `.cpp`。
- 新增头文件默认使用 `.hpp`。
- 文件名使用全小写加下划线。
- 文件名必须准确表达职责，例如 `deformable_suspension.cpp`、`pid_calculator.hpp`。
- 除非目录中已有稳定先例，否则不新增连字符文件名。

**10. Include 规则**

- 通用默认顺序：标准库、第三方库、项目内头文件。
- 不同组之间空一行。
- 本地相对路径头文件放最后。
- 但若当前目录已有更稳定局部先例，应先贴近局部风格。
- 修改代码后必须检查是否补全必要标准库头，不允许依赖“别的头碰巧带进来”。

**11. 命名空间规则**

- 优先使用嵌套命名空间写法，例如 `namespace rmcs_core::controller::chassis {`。
- 命名空间结束必须写注释。
- 头文件全局范围禁止 `using namespace`。
- `using namespace` 仅允许在局部作用域内谨慎使用。

**12. 命名规则**

- 类、结构体、枚举、类型别名使用 PascalCase。
- 普通变量、成员函数、局部函数使用 `snake_case`。
- 私有成员变量默认以 `_` 结尾。
- 布尔变量名必须体现状态，例如 `ready_`、`active_`、`enabled_`、`valid_`。
- 常量优先使用 `static constexpr`；若沿用协议/位掩码风格，可使用全大写下划线。

**13. 类内布局规则**

- 优先使用 `public`、`protected`、`private` 顺序。
- 对 RMCS 组件类，这是强约束，不是建议。
- 构造、析构、生命周期函数必须放前面。
- `before_updating()`、`before_pairing()`、`update()` 必须放在构造函数之后、`private:` 之前。
- 辅助函数放 `private`。
- 私有区优先按以下顺序组织：
  - 常量
  - 小型静态 helper
  - 私有辅助结构体
  - 业务辅助函数
  - `InputInterface` / `OutputInterface` 成员
  - 算法对象、配置、状态变量
- 复杂组件可以用少量内嵌 `Input` / `Output` struct 集中注册接口，但不是默认要求。
- 不要为了形式感额外引入大量只用一次的小结构体。
- 不允许把组件成员布局写成“先 private 成员，后 public 构造函数和 update”的风格。

**14. 函数与实现规则**

- 构造函数里优先做：参数读取、成员初始化、接口注册。
- 接口注册必须在构造函数体中直接可见，不能藏在 helper 中。
- `before_pairing()` 用于基于全局输出图做动态注册。
- `before_updating()` 用于配对完成后的检查、fallback、启动期初始化。
- `update()` 只做周期逻辑。
- `reset_all_controls()`、`update_*()`、`calculate_*()` 这类命名是当前仓库常见风格，可优先沿用。
- 能保持在一个函数里的逻辑，不要过度拆辅助函数。

**15. `.hpp` 使用规则**

- `.hpp` 适合：
  - 模板
  - 轻量工具类
  - 设备封装
  - UI widget
  - 小型算法类
- `.hpp` 使用 `#pragma once`。
- 头文件中可以内联实现简单函数，但避免把本应独立成组件的大块运行时逻辑塞进头文件。

**16. 注释规则**

- 只写必要注释：为什么、协议假设、线程语义、边界条件、单位、数学意图。
- 不写“给变量赋值”这种低价值注释。
- TODO 必须是真实待办，不允许空泛占位。
- 修改已有 `NOLINT`、协议说明、线程说明时，必须保留原语义。

**17. 现代 C++ 使用规则**

- 仓库接受现代 C++，但必须服务于可读性和必要性。
- 可使用 `constexpr`、`std::span`、`std::bit_cast`、`requires`、结构化绑定等现有先例。
- 不得为了“更现代”就大面积重写现有文件表达风格。
- 复杂模板、concepts、锁自由结构只有在确实需要时才引入。

**18. Auto Aim 特别规则**

- `rmcs_auto_aim_v2` 下优先遵守其子目录 `AGENTS.md`。
- 该目录 include 顺序规范与主工作区可不同。
- `RMCS_PIMPL_DEFINITION`、局部 `using namespace`、算法类命名等必须按其子项目规则处理。

**19. 禁止事项**

- 禁止无要求的大面积重命名。
- 禁止无要求的大面积格式化。
- 禁止保留明显的死代码、死参数、死配置。
- 禁止把本应是组件的逻辑继续藏在 detail 模块里。
- 禁止新增与当前目录风格不一致的新命名体系。
- 禁止 reset 时只关输出不清状态。

**20. AI 执行清单**

在 RMCS 中做 C++ 改动前后，至少检查：

- 当前目录已有风格是什么
- 组件职责是否清晰
- 输入输出接口是否明确
- `plugins.xml` 是否已同步
- YAML `components` 是否已同步
- 参数块是否放在正确组件名下
- 是否清理了不再使用的死参数
- reset 语义是否完整
- include 是否补全

**21. 一句话要求**

- 在 RMCS 中写代码时，必须像这个模块里长期维护该系统的工程师一样写，而不是引入一套新的个人风格。
