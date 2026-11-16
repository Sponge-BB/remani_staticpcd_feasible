# 当前工程快照（2026-11-16）

本仓库根目录对应实车 l 端 `/home/l/remani_ws/src`，包含当前 REMANI + MPPI 源码、驱动修改、launch、参数、消息定义、碰撞模型、测试及运行地图。`NEXTE_Sentry_Nav`、`NeuPAN-main` 和 `my_description/lidar/velodyne_simulator` 以普通源码目录收录；各自本机 Git 元数据不上传。

当前运行代码版本为 `20261116-232420-snapshot-resync`，修复地图完整帧/增量帧缓存失配后的重同步与错误分类。65 项离线检查、实际保存报文重放和规划节点重新编译通过；本轮修改尚未实车验证。地图计算开销及控制交付时序问题仍未全部解决。

ESDF 分块读写、nominal 候选分批验收两项性能试验未达到要求，没有写入当前运行源码。GitHub 快照不包含这些试验副本。

编译、部署产物、原始实验数据、缓存和本地大体积回退归档不上传。源码回退不能替代匹配的部署产物恢复，也不能恢复固件、CAN 状态或控制器持久参数。完整实车回退记录保留在 l 端 `/home/l/remani_ws/rollback/VERSIONS.tsv`。

恢复使用时，将这些包置于 ROS Noetic catkin 工作空间，重新编译并加载该工作空间环境；不要复用其他版本的 build 缓存。实车 Livox 驱动工作空间 `/home/l/ws_livox`、外部 Piper SDK、系统 ROS 依赖及硬件配置仍需单独准备。现有 launch 中的本机路径与模型内的历史绝对符号链接按当前工程原样保留，迁移到其他电脑时需要核对。
