# huananhu_sim_plugins 迁移记录

来源 `/home/liu/培训/kaohe/rmoss_gazebo/rmoss_gz_plugins`，只保留导航所需两个系统插件：

- `MecanumDrive2`：载体上的力/力矩直接驱动（对 `chassis` link 施加世界系 wrench，PID 目标为 link 线/角速度），因此 `low_profile_robot` 的 fixed 球形支撑轮不需要转动。源项目相对上游增加了 PID 与 `/odometry` 发布。
- `WheelTractionDrive`：`drive_model:=rollers` 时的被动滚子接触驱动，包含 `/wheel_odometry` 与 `/odometry` 真值旁路。

未迁移 `LightBarController`、`ProjectileShooter`：导航链不使用，且 rmul_2027 生成时源项目本就移除了 LightBar。

许可证：源 `package.xml` 为 Apache 2.0；`MecanumDrive2.cc` 头部为 Apache 2.0（RoboMaster-OSS）。`WheelTractionDrive.cc` 无许可证头，迁移时保留源文件原样，待上游确认。
