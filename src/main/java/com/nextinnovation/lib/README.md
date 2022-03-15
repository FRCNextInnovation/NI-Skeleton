# NI-Code-Lib-J
[![Next-Innovation](https://img.shields.io/badge/Next-Innovation-blueviolet?style=flat)](https://github.com/FRCNextInnovation) [![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat)](https://github.com/RichardLitt/standard-readme) [![Lang](https://img.shields.io/badge/Lang-zh--CN-Green?style=flat)]()

## Packages

- [`com.nextinnovation.lib.controllers`](controllers)

  控制器算法包，包含全部的控制器算法，如用于 nonholonomic 机器人底盘的 `HeadingController`，以及常用的`PIDF`控制器算法。

- [`com.nextinnovation.lib.drivers`](drivers)

  驱动包，用于放置对机器人上的一些高频率使用或者原生接口不够方便的设备进行过再封装的类。例如`LazyTalonFX`、`Limelight	`等设备。

- [`com.nextinnovation.lib.experiments`](experiments)

  在每个赛季的开发过程中也许会有新的算法或者封装好的代码被加入到 Lib 中，在赛季正式结束之前，为了方便后续的整理工作，我们首先把新加入的算法加入到该包下，后续再进行 reveal 和合并。

- [`com.nextinnovation.lib.geometry`](geometry)

  由[Team254](https://github.com/Team254) 团队所开发的几何运算库，提供了舒适合理的接口，方便和加速了我们的开发工作。

- [`com.nextinnovation.lib.io`](io)

  用于存放常用的io设备，例如我们最喜欢的`StatefulXboxController`。

- [`com.nextinnovation.lib.kinematics`](kinematics)

  用于存放所有的正向/逆向运动学解算类。

- [`com.nextinnovation.lib.math`](math)

  用于存放所有的数学算法。

- [`com.nextinnovation.lib.spline`](spline)

  用于存放所有的曲线类型以及曲线生成器，如`CubicHermiteSpline`和`QuinticHermiteSpline`等。

- [`com.nextinnovation.lib.trajectory`](trajectory)

  由[Team254](https://github.com/Team254) 团队所开发的轨迹生成算法，提供了方便强大的轨迹生成接口，简化了我们的轨迹跟踪工作。
  
- [`com.nextinnovation.lib.utils`](utils)

  存放所有的工具方法。