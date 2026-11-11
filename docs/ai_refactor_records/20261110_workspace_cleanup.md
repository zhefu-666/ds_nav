# 项目日志实验与历史报告清理记录

## User Intent
清理已列明的非功能残留，并说明与既定目标的差距。

## Scope
删除可再生log/pytest/Python缓存和截图；归档artifacts、旧审计/任务/规划报告、.mimosa、两个临时探针及protocol.hpp.bak；更新保留文档链接、新增PROJECT_STATUS。本次只保留这一份当前清理记录，旧记录已归档。

## Out of Scope
不改运行源码、参数、地图、几何、测试、build/install、Git；不构建、不跑运动试验。

## Explorer Findings
### Files inspected
清理候选清单、git状态、运行进程快照、启动链路引用、符号链接、旧六大方向文档、当前仿真README。
### Active logic path
scripts与src及install提供启动链路，未发现指向artifacts的运行符号链接。
### Data flow
历史实验和证据整体同文件系统rename到/home/liu/ds_nav_archive/20261110_cleanup/；不丢弃验收原始数据。旧脚本硬编码工作空间路径，归档后须恢复路径方可复现。
### Risk notes
用户授权清理优先；不按未跟踪状态批量删除。protocol备份内容不同，归档而非永久删除。缓存和log删除不可恢复，但均为可再生材料。归档不释放总磁盘空间。
### Recommended modification boundary
仅清理白名单，保留运行文件并检查启动预检。

## Modifier Changes
### Files changed
26项文件/目录移出项目，17项缓存/log路径删除；更新README、仿真README、技术概述中的归档链接，新增docs/PROJECT_STATUS.md及本记录。
### Key changes
项目约8.4GB降至1.3GB；归档普通文件约7.08GiB，du归档占用约7.2G。manifest含源/目标路径，protected_files_sha256含1352个源码哈希。
### Behavior preserved
1352个保护源码哈希一致，运行参数/二进制未主动改写，build/install保留。
### Behavior intentionally adjusted
仅实验/历史文件位置及缓存内容改变，历史脚本需要恢复依赖路径后复用。
### Notes
归档清单：/home/liu/ds_nav_archive/20261110_cleanup/cleanup_manifest.json。

## Auditor Review
### Checks performed
- [x] 1352个源码哈希一致
- [x] 已归档源路径残余0
- [x] 安装/启动预检通过：./scripts/huananhu_sim.bash --headless --check-install
- [x] 文档链接检查：归档链接存在；原技术概述的文件:行号为应用支持链接，非缺失文件
- [x] git diff --check
- [x] 未改XML/launch/YAML
- [ ] 构建授权（本次不构建）
### Issues found
旧复现脚本不是可直接运行的归档入口，已在README明确。完整功能与研发目标差距见PROJECT_STATUS。
### Final result
PASS（清理与启动预检）；不代表新增运动或实车验收。

未执行构建：AGENTS.md 禁止在未获用户明确许可前运行构建命令。
