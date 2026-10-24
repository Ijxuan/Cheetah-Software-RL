# HIM ONNX 导航爬楼梯接入

分支 `himloco-onnx-nav-stairs` 基于 `himloco-onnx-deploy`。导航只提供速度，
HIM 的 ONNX、观测历史、动作映射、PD、推理周期及故障 OFF 链路保持原有实现。
本次没有加入旧平地控制器或自动地形/控制器切换。

## 导航发送端协议

- LCM URL：`udpm://239.255.76.67:7667?ttl=255`，对应现有 HardwareBridge。
- 消息类型：`lcm-types/cmd_vel_msg.lcm`，两个 double：`vx`、`wz`。
- 通道任选一个：`cmd_vel`、`cmd_vel_msg`、`CMD_VEL_LCM`；只运行一个指令发布源。
- `vx` 为机体前向速度（m/s），`wz` 为偏航角速度（rad/s），侧向速度固定为零。
- 连续发送，建议 20–50 Hz；停止运动时继续发送零速度。
- NaN/Inf 被拒绝且不刷新有效消息时间。非导航状态的消息被丢弃。
- 导航限幅分别为 `vx ±0.8 m/s`、`wz ±0.8 rad/s`。这只是接口限幅，
  首次上楼使用同一策略已通过手动测试的速度，规划器应设置更窄的适用范围。

当前消息没有发送时间戳/序号，接收端超时以本机 steady_clock 接收时间计；
不能识别网络中延迟送达的旧包，发送端不要积压或重放命令。

## 操作与状态

保持 `use_rc: 1`，沿用原有启动程序、策略路径和起立流程：

1. LT+A 起立，BACK（代码按钮编号 7）进入 RL_JOINT_PD。
2. 按下 RB 进入导航等待状态；此前的命令不复用，需要收到新消息才运动。
3. 长按 RB 不会反复清空命令。导航速度由每次 RobotRunner 读取 RC 时更新，
   即使手柄没有事件也会变化。
4. RT 超过 15000 立即切回当前手柄速度；RT 与 RB 同时按下时 RT 优先。
5. 最后一条有效消息满 1 秒后输出零速度；3 秒后取消自动恢复并保持零速度。
   此时再次来包不会恢复运动，需松开再按 RB，或按 RT 接管手动。
   从 RB 进入后始终未收到消息也执行同样超时。
6. LT+B、控制器安全故障请求 OFF、退出 RL 模式均取消导航。
   恢复起立/RL 不会自动恢复导航，需要新的 RB 按下沿。

终端只在采样观察到状态变化时打印 `[Navigation]` 状态。
默认开启的 `rl_robot_state.command` 可用于观察经控制器限幅/死区后的最终输入。
零速度是 HIM 的停步指令，不是机械锁止，也不等于关节失能。

超时和接口限幅位于 `robot/include/rt/NavigationCommand.h`，修改后重新编译。
单个 RC 互斥锁保护导航状态、手动缓存和模式；控制端只覆盖输出快照，
不把导航速度写回手动缓存，不存在超时清理函数重复加锁。

## 编译和离线测试

在仓库根目录，复用已经配置好 ONNX Runtime 的 build-onnx：

```bash
cmake -S . -B build-onnx
cmake --build build-onnx --target mit_ctrl navigation_command_test -j2
(cd build-onnx && ctest --output-on-failure -R 'navigation_command|legged_gym_rl_policy_config|himloco_onnx_policy_inference')
```

如果终端已位于 build-onnx：

```bash
cmake ..
cmake --build . --target mit_ctrl navigation_command_test -j2
ctest --output-on-failure -R 'navigation_command|legged_gym_rl_policy_config|himloco_onnx_policy_inference'
```

首次配置、ONNX Runtime 安装和 NO_SIM 选项沿用 README 的构建说明。
测试使用模拟时间覆盖边界，再使用管道模拟真实 joystick 事件调用生产 RC 接口，
验证独立更新、限幅、无效数据、长按、接管、实际 1/3 秒超时和 OFF 恢复。
测试不启动硬件桥、不发布 LCM 运动指令、不驱动电机。

## 上机器人前的验证顺序

先固定 ONNX 文件并记录其 SHA256；更新策略文件需要重启控制程序。
先在平地验证导航方向、RT 接管和断流，再用导航发送器复现已验证的直线上楼速度，
最后连接完整规划器。对照发送速度、`rl_robot_state.command`、实际速度和姿态，
确认规划器不会在楼梯入口停车或产生未经验证的连续转向。
编译与离线测试不能替代真实网络联调及实机楼梯验证。
