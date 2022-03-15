# NI-Code-Template-2022
[![Next-Innovation](https://img.shields.io/badge/Next-Innovation-blueviolet?style=flat)](https://github.com/FRCNextInnovation) [![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat)](https://github.com/RichardLitt/standard-readme) [![Lang](https://img.shields.io/badge/Lang-zh--CN-Green?style=flat)]()

NI-Code-Template-2022 是 NI 电控组用于团队 8214 与 8583 的代码开发工作而特别整理的代码模板。此模板由 Java 开发并基于 [WPILib's Java Control System](https://docs.wpilib.org/)。

本 Readme 包含NI代码模板的主要设计思路，以及各个 Package 与重点 Class 的相关介绍。

## 使用流程

1. clone 该仓库。
2. 使用 [IntelliJ Idea](https://www.jetbrains.com/zh-cn/idea/) 打开仓库( 必须至少安装 [FRC 插件](https://plugins.jetbrains.com/plugin/9405-frc))。
3. 执行 Build Robot 以验证项目配置是否成功。
4. 开始开发。

## 设计思路

我们用状态机思路设计子系统，但是将每个状态的具体实现在子系统Class的外部实现。例如对于一个`Intake`，其文件夹下可能有`Intake.java `与`states`包，states包下会有`IntakeDisabled`，`IntakeManual`，`IntakeAuto`等 State 对`Intake`在不同状态下的行为模式进行实现。 

## Package 与 Class 的介绍

- [`com.nextinnovation.team8214`](src/main/java/com/nextinnovation/team8214)

  [`Constants.java`](src/main/java/com/nextinnovation/team8214/Constants.java)内包含机器人本赛季的全部常量([`Constants.java`](src/main/java/com/nextinnovation/team8214/Subsystems)内控制器的各种参数，[`TrajectortGenerator`]()和[`DriveMotionPlanner`](src/main/java/com/nextinnovation/team8214/planners/DriveMotionPlanner.java)等重要 Class 的全部参数)，[`Ports`](src/main/java/com/nextinnovation/team8214/Ports.java)内包含机器人类各个气动组件端口号，传感器端口号，以及 CAN 设备 ID号等数据。在[`Robot`](src/main/java/com/nextinnovation/team8214/Robot.java)内，我们进行全部的初始化工作，并控制各个[`Looper`]()在比赛中的不同环节的启动与关闭。值得注意的是我们把Log工作用单独的`logLooper`进行控制，以防止在log过程中意外产生的阻塞导致机器人的控制阻塞。

- [`com.nextinnovation.team8214.auto`](src/main/java/com/nextinnovation/team8214/auto)

  解决比赛在自动阶段时，机器人自动方案的选择，以及各个子系统的动作。包括[`modes`](src/main/java/com/nextinnovation/team8214/auto/modes)与[`actions`](src/main/java/com/nextinnovation/team8214/auto/actions)包。

- [`com.nextinnovation.team8214.devices`](src/main/java/com/nextinnovation/team8214/devices)

  用于存放机器人中的各种设备，`device`的特点是具有实体且不存在状态机需求的小型单元，例如每个机器人中都常有的[`AHRS`](src/main/java/com/nextinnovation/team8214/devices/ahrs)、[`PneumaticCompressor`](src/main/java/com/nextinnovation/team8214/devices/PneumaticCompressor.java)等，一般为某种信息的提供者。

- [`com.nextinnovation.lib.loops`](src/main/java/com/nextinnovation/team8214/loops)

  在`Command Base`结构中，`Command`的主要执行器为`Command Schedular`，在NI代码框架中，我们使用[`Looper`](src/main/java/com/nextinnovation/team8214/loops/Looper.java)作为机器人程序的主要执行器，[`Looper`](src/main/java/com/nextinnovation/team8214/loops/Looper.java)将会以 50hz 的速度执行[`Loop`](src/main/java/com/nextinnovation/team8214/loops/ILoop.java)，其用来描述一个控制回路在`onStart`、`onLoop`、`onStart`三个不同阶段的行为。一般的，每一个`Subsystem`都有一个自己的`Loop`。

- [`com.nextinnovation.team8214.managers`](src/main/java/com/nextinnovation/team8214/managers)

  包含所有的`manager`，用于对多个相同或具有相同特性设备或数据进行管理，如TalonSRX、Joystick(手柄)、Coordinate Frame(坐标系)。

- [`com.nextinnovation.lib.planners`](src/main/java/com/nextinnovation/team8214/planners)

  `planner`用于对机器人的各种行为进行规划与优化，例如[`DriveMotionPlanner`](src/main/java/com/nextinnovation/team8214/planners/DriveMotionPlanner.java)依照轨迹输入，对机器人底盘的运动提供优化与输入。

- [`com.nextinnovation.team8214.subsystems`](src/main/java/com/nextinnovation/team8214/subsystems)

  包含各个[`BaseSubsystem`](src/main/java/com/nextinnovation/team8214/subsystems/BaseSubsystem.java)的实现类以及每个子系统的`states`，每个子系统按照状态机类型进行设计，分离存放子系统提供的接口与各个子系统在不同状态下的行为模式。

- [`com.nextinnovation.lib`](src/main/java/com/nextinnovation/lib)

  NI 电控组开发所使用的软件库，合并了 [Team254](https://github.com/Team254) 的 lib。

