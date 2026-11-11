# huananhu_sim_resources 资源清单

来源项目：`/home/liu/培训/kaohe`（只读迁移，目标运行不依赖该路径或其 install）。

| 目标路径 | 来源 | 说明 |
|---|---|---|
| `models/rmul_2027` | `rmu_gazebo_simulator/resource/models/rmul_2027` | RMUL 2027 场地网格与碰撞代理 |
| `worlds/rmul_2027_world.sdf` | `rmu_gazebo_simulator/resource/worlds/rmul_2027_world.sdf` | 场地位姿 `[0, 1, 0.5, π, 0, 0]` 原样保留 |
| `ign/gui.config` | `rmu_gazebo_simulator/resource/ign/gui.config` | Fortress GUI 配置 |
| `models/low_profile_robot/model.sdf` | `pb2025_robot_description/resource/models/low_profile_robot/model.sdf` | 低矮载体；本包改动见下 |
| `models/huananhu_demo` | `huananhu_kinematic_demo/models/huananhu_demo` | 339 个 CAD STL、关节；2026-10-29 新增 14 个凸包碰撞体 |
| `models/huananhu_demo/collision/*_hull.obj` | `huananhu_kinematic_demo/models/huananhu_demo/collision/` | 14 个非轮组 CAD 凸包，逐顶点法线，替换旧组盒 |
| `urdf/huananhu.urdf` | `huananhu_kinematic_demo/urdf/huananhu.urdf` | 网格 URI 由 `package://huananhu_kinematic_demo` 改为 `package://huananhu_sim_resources` |
| `urdf/carrier.urdf` | 新写 | 低矮载体的 RSP 描述，帧与 SDF 一致 |
| `scripts/navigation_model.py` | `huananhu_kinematic_demo/scripts/navigation_model.py` | 组合 CAD 与载体；合并凸包载入与 URDF 凸包，保留雷达真实遮挡 |
| `scripts/roller_wheels.py` | `huananhu_kinematic_demo/scripts/roller_wheels.py` | 被动滚子轮（`drive_model:=rollers`） |
| `scripts/kinematic_math.py` | `huananhu_kinematic_demo/scripts/kinematic_math.py` | 关节/轮速映射，姿态控制器导入 |
| `config/model.json` | `huananhu_kinematic_demo/config/model.json` | 关节、轮组、几何参数 |
| `config/tunnel_profile.json` | `huananhu_kinematic_demo/config/tunnel_profile.json` | 低姿态唯一配置：fold=0、Yaw=0、aim=+4°，高 244.469 mm |
| `config/cad_support_vertices.json` | `huananhu_kinematic_demo/config/cad_support_vertices.json` | CAD 支撑顶点，足迹与实测高度共用 |
| `config/linkage_geometry.json` | `huananhu_kinematic_demo/config/linkage_geometry.json` | 连杆闭合校验参考 |
| `worlds/tunnel_spec_250_world.sdf` | `rmu_gazebo_simulator/resource/worlds/tunnel_spec_250_world.sdf` | 250 mm 净空、800 mm 宽、4.06 m 平地验证场景 |
| `worlds/tunnel_spec_235_world.sdf` | `artifacts/tunnel_spec_20261029/tunnel_spec_235_world.sdf` | 仅负例测试世界，不默认使用 |

## 相对源项目的改动

1. `models/low_profile_robot/model.sdf`
   - 2D 雷达 `ignition_frame_id` 由 `chassis` 改为 `front_laser`，测量坐标系单独定义。
   - IMU 增加 `<pose>0 0 0 0 0 0</pose>` 与 `ignition_frame_id=body`，频率 100→200 Hz，与 Point-LIO 的 `imu_time_inte=0.005` 对齐。
2. `urdf/huananhu.urdf` 的 `package://` 前缀改为本包名。
3. 新增 `urdf/carrier.urdf`：`base_footprint→chassis→{body, front_laser, navigation_laser, 四轮}`，全部 fixed。
4. 2026-10-29 迁移：
   - 14 个非轮组 link 的旧组盒 collision 替换为 `collision/<link>_hull.obj` 凸包，缺失即报错；
   - URDF collision 同步改为同一凸包，RViz 与物理模型一致；
   - **未采用**源工程 GPU 雷达 `visibility_mask=2`；ds_nav 保留 CAD 对雷达射线的真实遮挡；
   - 滚子数量、轮毂扭矩上限 1.2 Nm、雷达/IMU 外参与命名均未改。
5. 低姿态高度 244.469 mm、横向包络 752.575 mm 由 `scripts/check_sim_clearance.py`
   以 501 组姿态与 14 个凸包静态复核；不代表实车编码器标定。

## 迁移已知限制（未在本阶段解决）

- 3D 雷达与 IMU 仍刚性固定在载体 `chassis` 上，不在 CAD `yaw1` 上；E2 的跨关节状态恢复和 `yaw1` 安装外参尚未实现。
- STL 许可证：`huananhu_kinematic_demo/package.xml` 原文为 `TODO`，不是经过 SPDX 校验的对外许可。用户 2026-10-28 已确认可迁入本仓库，仅限本队内部使用；本包不得对外声称这些网格是开源许可。
- `rmoss_gz_resources`（19 MB 模块化装甲/弹丸）未迁入，导航不需要。
