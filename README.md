# NI-Code-Template-Ultimate
[![Next-Innovation](https://img.shields.io/badge/Next-Innovation-blueviolet?style=flat)](https://github.com/FRCNextInnovation) [![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat)](https://github.com/RichardLitt/standard-readme) [![Lang](https://img.shields.io/badge/Lang-zh--CN-Green?style=flat)]()

NI-Code-Template-Ultimate是 NI 电控组在旧代码模板的基础上，为团队 8214 与 8583 的代码开发而特别整理的代码模板。此模板由 Java 开发并基于 [WPILib's_Java_Control_System](https://docs.wpilib.org/)

本 Readme 包含NI代码模板的主要设计思路，以及各个 Package 与重点 Class 的相关介绍。

## 使用流程

1. clone 该仓库。
2. 使用 [IntelliJ_Idea](https://www.jetbrains.com/zh-cn/idea/) 打开仓库(必须至少安装 [FRC 插件](https://plugins.jetbrains.com/plugin/9405-frc))。
3. 执行 Build Robot 以验证项目配置是否成功。
4. 开始开发。

## 主要改进

- `disabledLooper`被取消。 
- `BaseSubsystem`内加入了计时功能，每个`Subsystem`将自动计算自己单次`Loop`的耗时。
- 从现在开始，每个`Looper`的频率都可以自定义 。
- 曾经位于`Robot`级下的一些基类和执行器类被挪入`Lib`，以增强其可复用性 。
- 原始的编程模板被简化，现在每一个具体的`subsystem`由`Subsystem`、`SubsystemState`和`SubsystemConfig`组成，后两者视实际情况可选。
- `Subsystem`针对不同`State`的控制逻辑被挪入`Subsystem`内部实现，目的是减少嵌套和重入次数，并提高类的线程安全性。
- `Vision`被作为`Subsystem`进行了重写。
- 曾经的`Constants.java`被重命名为`Config.java`，并拆分到了具体的`subsystem`，现在`Config.java`只存储`Robot`级的数据。
- 场地数据被挪入到了`Field.java`。
- `ControlFlagManager`重命名为`ControlSignalManager`，加入`SlewRateLimiter`以顺滑`Swerve`的驾驶。
- `TrajectoryGenerator`更改为静态类，其中的`TrajectorySet`被单独提出为一个新的类。
- `TimingUtil`的接口改进，`MAX_DECEL`和`SLOWDOWN_CHUNKS`被取消。
- `SwerveDriveModule`中引入了`CarpetScrubFactor`，以改善轮氏里程计的精度。
- `SwerveInverseKinematics`重写，从现在开始将只支持常量数据的运算，并将针对`SwerveDriveModuel`的最大平移速度进行约束。

## Package 与 Class 的介绍

- [`com.nextinnovation.team8214`](src/main/java/com/nextinnovation/team8214)

  我们始化工作，并控制各个[`Looper`]()在比赛中的不同环节的启动与关闭。

- [`com.nextinnovation.team8214.auto`](src/main/java/com/nextinnovation/team8214/auto)

  解决比赛在自动阶段时，机器人自动方案的选择，以及各个子系统的局部动作的执行。包括[`modes`](src/main/java/com/nextinnovation/team8214/auto/modes)与[`actions`](src/main/java/com/nextinnovation/team8214/auto/actions)包。

- [`com.nextinnovation.team8214.devices`](src/main/java/com/nextinnovation/team8214/devices)

  用于存放机器人中的各种设备，`device`是具有实体且不存在状态机需求的小型电子单元，例如[`AHRS`](src/main/java/com/nextinnovation/team8214/devices/ahrs)、[`PneumaticCompressor`](src/main/java/com/nextinnovation/team8214/devices/PneumaticCompressor.java)等。

- [`com.nextinnovation.team8214.managers`](src/main/java/com/nextinnovation/team8214/managers)

  包含所有的`manager`，用于对多个相同或具有相同特性设备或数据进行管理，如Joystick(手柄)、Coordinate Frame(坐标系)、Fused Odometer(融合里程计)等。

- [`com.nextinnovation.team8214.subsystems`](src/main/java/com/nextinnovation/team8214/subsystems)

  包含各个[`BaseSubsystem`](src/main/java/com/nextinnovation/lib/subsystems/BaseSubsystem.java)的实现类以及每个子系统的`state`，每个子系统按照状态机类型进行设计，子系统通过状态决定自己的行为模式。

- [`com.nextinnovation.lib`](src/main/java/com/nextinnovation/lib)

  NI 电控组开发所使用的软件库，合并了 [Team254](https://github.com/Team254) 的 lib。

