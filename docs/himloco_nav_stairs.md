# HIM ONNX 导航爬楼梯接入

分支 `himloco-onnx-nav-stairs` 在上层 `ControlFSM` 中注册两个独立状态：
Start 请求 `RL_FLAT`（54），Back 请求 `RL_STAIRS`（53）。原 `RL_JOINT_PD=53`
保留为楼梯模式的兼容名称。导航只提供速度，切换时先停稳再交接，并保留导航
控制权。没有自动地形识别。

## FSM 状态与共享数据

- `FSM_State_RLFlat` 和 `FSM_State_RLStairs` 是两个独立状态对象，分别执行平地
  和楼梯策略，定义在 `FSM_State_RLPolicies.h`。`ControlFSM.currentState` 实际切换。
- `ControlFSM` 持有 `RLControlContext`，两个状态共享它。上下文保存模型、
  两套历史、最后下发目标、交接进度和公共保护；它不继承 `FSM_State`。
- 旧状态在 `run()` 中给零速并检查停稳，交接准备就绪后仍保持旧状态的控制权。
  其 `checkTransition()` 才返回目标状态，随后上层 FSM 调用旧状态的
  `transition()/onExit()` 和新状态的 `onEnter()`。
- 新状态 `onEnter()` 接管实际目标与两套历史，开始目标和增益过渡。
  跨 RL 状态不会清空缓存或插入失能帧；退出到恢复站立、Passive 或 OFF 时才清理。
- `control_mode`/RC mode 表示请求模式；停稳等待或超时时，请求可能是 53 而
  实际仍在 54。`currentState`、`[FSM] Enter/Exit` 日志和 `rl_robot_state.mode`
  反映实际激活的状态，后者从共享上下文的已激活身份生成。
- `RobotRunner` 的 OFF 路径会调用 `ControlFSM.initialize()`；此入口也会执行
  当前 RL 状态的退出清理，避免急停打断交接后重新使用旧历史。

## 两套策略与历史

| 项目 | Start：平地 | Back：楼梯 |
| --- | --- | --- |
| FSM 状态 / 模式号 | `RL_FLAT` / 54 | `RL_STAIRS` / 53 |
| 来源 | `origin/local-deploy-nav-integration`，提交 `f2c356431cabe54791a7db034bcfbd74174125e1` | 当前 HIM ONNX |
| 模型 | `actor_5000.txt`、`estimator_5000.txt`、`mean5000.csv`、`var5000.csv` | `rl-checkpoints/onnx/policy.onnx` |
| 观测 | 141 维，拼接 11 维估计器输出 | 六帧 45 维，最新帧在前 |
| 周期 | 10 ms | 20 ms |
| 动作 | `q_init + 0.1 * actor_output`，机器人关节顺序 | `q_default + 0.25 * action`，HIM 关节顺序 |
| 实机增益 | Kp=17、Kd=0.4 | Kp=20、Kd=0.5 |

两套模型启动时加载，不在切换时读文件。平地目录由 CMake `RL_FLAT_MODEL_DIR`
指定，默认源码根目录 `actor_model`；楼梯沿用 `RAPID_RL_CHECKPOINT_DIR/FILE`。
平地加载器校验权重数量、归一化数量及有限性，拒绝损坏文件。

共享状态每 10 ms 采集一次，包括机器人顺序的 q/qd、姿态、角速度、足端位置、
速度指令和实际下发关节目标。平地保留最近六个真实采样；HIM 每两个采样更新
自己的六帧历史。在两个 RL 状态期间，两套历史持续更新，只有 `currentState`
对应的策略执行网络推理和关节输出。
初次进入 RL 保持当前关节目标并给零速至少 120 ms，等待两套历史就绪。

平地保持旧观测顺序、归一化、前两次关节目标和误差/速度历史的 5/3/1 帧延迟抽样。
未激活 HIM 的上一动作由实际目标反算，不能使用未执行的后台网络预测；HIM 正常
激活时继续使用自己的原始上一动作，交接过程中使用实际目标反算值。
两者的腿序、默认姿态、缩放和历史缓存独立。HIM 推理接口与历史更新分开，避免
一次控制回合滚动历史两次。采样中断不补造历史，历史就绪前不运行对应网络。

状态估计统一由 RobotRunner 更新。旧平地的接触提示只在平地激活时设置，供下一
回合估计器使用，不再在策略里重复运行估计器；退出平地时清回默认 0.5。
这与旧分支重复运行估计器的时序存在区别，需在仿真和实机上验证其行为。

模型 SHA256（接入时的实际文件）：

```text
6ce432aaa92ef770c16fb0130b3abedb7eeddae9b96195ade5b6d5f131905bbc  actor_5000.txt
35aeba084eff4306002a197bcfdbcae5f3b7160914134ff14232396e919daeff  estimator_5000.txt
7e37ec960bb8e73f706a64dc9ce47a08cd000d2ec752dda3b29bfe7378dd90c0  mean5000.csv
cecbfc290f49e64b6e9b15ef00a1025c0171fdedfca98aa6145445d3b27ff635  var5000.csv
e1bcfdfd2469fed0ec8d9f36e866b6941cdfd7af3e87763efdfb78dae784871a  policy.onnx
```

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

1. LT+A 起立，Start（编号 6）进入平地，Back（编号 7）进入楼梯。OFF 时单按
   Start/Back 无效。LT+X 返回恢复站立；Start 不再承担恢复站立功能。
2. 按下 RB 进入导航等待状态；此前的命令不复用，需要收到新消息才运动。
3. 长按 RB 不会反复清空命令。导航速度由每次 RobotRunner 读取 RC 时更新，
   即使手柄没有事件也会变化。
4. RT 超过 15000 立即切回当前手柄速度；RT 与 RB 同时按下时 RT 优先。
5. 最后一条有效消息满 1 秒后输出零速度；3 秒后取消自动恢复并保持零速度。
   此时再次来包不会恢复运动，需松开再按 RB，或按 RT 接管手动。
   从 RB 进入后始终未收到消息也执行同样超时。
6. LT+B、控制器安全故障请求 OFF、退出 RL 模式均取消导航。
   恢复起立/RL 不会自动恢复导航，需要新的 RB 按下沿。

在两个 RL 状态间切换时，当前策略收到零速度指令，导航仍持续接收和计时。
切换姿态条件只有两项：机身角速度模长 `<0.2 rad/s`，以及切入平地时总倾角 `≤30°`
（切入楼梯不增加此倾角门槛）。条件连续满足 200 ms，并且两套历史就绪，才交给目标策略。
不再要求机身线速度或关节速度低于阈值，因此允许尚有平移或腿部运动时切换；
有限值检查、模型可用性和原有急停保护仍保留。这里的“停稳”仅指上述姿态条件。目标角与增益用 500 ms
过渡，交接时每次策略输出的目标角变化额外限制为 0.05 rad；若仍被该限制截断，
延长交接直到目标跟上。交接完成才恢复最新的有效导航/手动速度。

5 秒内不能停稳，或请求的模型不可用，会保持当前控制器零速并打印 `blocked`，
需要重新按 Start/Back 重试。选择当前控制器可取消尚未交接的请求。长按不重复
请求；交接过渡中的新选择排队，等过渡完成再处理。切换期间导航超时仍生效，
超时后不会因为切换完成而自行恢复。RT 仍只负责导航转手动，不跳过控制器停稳
和交接流程。LT+B 和安全故障始终可以中止流程。

两套策略共用最终关节限幅、目标偏差告警、推理硬超时以及 OFF/失能链路。
HIM 保持 roll/pitch 各 45° 的姿态保护；平地额外保留旧分支总倾角 30° 的保护，
交接期间也执行该保护。请求转到平地时须先满足其倾角范围才计入停稳时间。
平地实机 PD 使用旧分支 17/0.4，HIM 使用当前 20/0.5，切换时随目标一起过渡。
仿真 GUI 的 `rl_kp_joint/rl_kd_joint` 仅覆盖 HIM 仿真增益。
`[DualRL] active=flat/stairs requested=... state=...` 标明当前策略和交接阶段；
`rl_robot_state.mode` 平地为 54、楼梯为 53，两种策略均以 HIM 关节顺序发布 q/qd/last_action，
平地的 last_action 是目标换算值，不能当作旧平地 actor 的原始输出。

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
cmake --build build-onnx --target mit_ctrl navigation_command_test dual_rl_policy_test dual_rl_controller_test rapid_rl_policy_config_test rapid_rl_policy_benchmark -j2
(cd build-onnx && ctest --output-on-failure -R 'navigation_command|legged_gym_rl_policy_config|himloco_onnx_policy_inference|dual_rl_')
```

如果终端已位于 build-onnx：

```bash
cmake ..
cmake --build . --target mit_ctrl navigation_command_test dual_rl_policy_test dual_rl_controller_test rapid_rl_policy_config_test rapid_rl_policy_benchmark -j2
ctest --output-on-failure -R 'navigation_command|legged_gym_rl_policy_config|himloco_onnx_policy_inference|dual_rl_'
```

首次配置、ONNX Runtime 安装和 NO_SIM 选项沿用 README 的构建说明。
测试使用模拟时间覆盖边界，再使用管道模拟真实 joystick 事件调用生产 RC 接口，
验证独立更新、限幅、无效数据、长按、接管、实际 1/3 秒超时和 OFF 恢复。
测试不启动硬件桥、不发布 LCM 运动指令、不驱动电机。
`dual_rl_policy` 对照旧平地的观测布局及模型输出，并验证 HIM 历史更新与推理拆分
前后的输出一致性。`dual_rl_controller` 用真实模型和注入的时间/机器人状态调用
生产 `ControlFSM.runFSM()`，覆盖真实状态身份、跨状态历史连续性、两个方向的
交接、停稳超时、重试、增益、交接中恢复站立/OFF 中断以及 GUI 模式 53/54。
测试链接与 `mit_ctrl` 相同的 `mit_controller_core`，注入状态注册表以避免启动
无关的视觉订阅线程；RL/恢复站立/Passive 状态和 FSM 调度均使用生产代码。
该测试关闭虚拟机器人调试状态发布，但现有 FSM 基类初始化 LCM 仍要求环境允许
本地 multicast 套接字；网络权限受限的沙箱需在允许该套接字的环境运行。

## 上机器人前的验证顺序

先固定 ONNX 文件并记录其 SHA256；更新策略文件需要重启控制程序。
先在平地验证导航方向、RT 接管和断流，再用导航发送器复现已验证的直线上楼速度，
最后连接完整规划器。对照发送速度、`rl_robot_state.command`、实际速度和姿态，
确认规划器不会在楼梯入口停车或产生未经验证的连续转向。
编译与离线测试不能替代真实网络联调及实机楼梯验证。
