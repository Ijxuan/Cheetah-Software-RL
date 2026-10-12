# Mini Cheetah ONNX 控制器

来源：`Oct12_09-14-06_smoke_minich/model_1000.pt`，iteration 1000。
文件：`policy.onnx`。ONNX opset 11，固定 batch=1，float32。
包含 HIM encoder、16 维 latent 的 L2 归一化和 actor，输出确定性动作均值。

参考 hunter130 的 `hunter130_train/legged_lab/scripts/play.py`，把部署推理网络导出到训练目录的 exported 下；使用单个输入和输出，供 ONNX Runtime 加载。
Mini Cheetah 的接口为 270 → 12，接入时需要使用下面的 Mini Cheetah 配置。

## 接口

| 名称 | 形状 | 含义 |
|---|---|---|
| `obs` | `[1, 270]` | 连续 6 帧观测，每帧 45 维，最新帧在最前 |
| `actions` | `[1, 12]` | 原始动作，关节顺序见下方 |

每帧观测按当前 `LeggedRobot.compute_observations()` 构建：

| 索引 | 内容 | 输入网络前的缩放 |
|---|---|---|
| 0:3 | 指令 vx、vy、yaw rate | `[2.0, 2.0, 0.25]` |
| 3:6 | 机体系角速度 | `0.25` |
| 6:9 | 机体系重力方向（水平时 `[0,0,-1]`） | `1.0` |
| 9:21 | 关节位置减默认位置 | `1.0` |
| 21:33 | 关节速度 | `0.05` |
| 33:45 | 上一控制周期经过裁剪的网络动作 | `1.0` |

以上缩放和观测裁剪 `[-100,100]` 由调用方完成；网络不维护历史。
物理重置时用第一帧重复填充全部 6 帧，之后每周期前插新帧。
指令阶段切换时继续维护原历史。

关节顺序已在导出时用当前 URDF 经 Isaac Gym CPU PhysX 加载并核对：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

默认角度按上述顺序（rad）：

```text
[0.1, -0.8, 1.62, -0.1, -0.8, 1.62,
 0.1, -0.8, 1.62, -0.1, -0.8, 1.62]
```

调用方裁剪输出到 `[-100,100]`，再计算
`q_target = q_default + 0.25 * actions`。
当前训练配置是 50 Hz 策略 / 200 Hz PD（dt=0.005，decimation=4），
hip/thigh 的 Kp/Kd 为 17/0.4，calf 为 34/0.8。
这些部署参数来自导出时当前源码，不是 checkpoint 保存的配置快照。
ONNX 输出不包含 PD、扭矩限幅、实机坐标转换或电机驱动。

## 验证

`policy.validation.json` 记录来源路径、SHA256、库版本和误差。
JIT 的 actor/encoder 参数与 checkpoint 逐项完全一致；
与原始 `HIMActorCritic.act_inference()` 的输出误差为 0。
ONNX checker 与 ONNX Runtime CPU 验证通过，共 262 组输入，包含零输入、
站立/移动/转向重复帧及 0.1/1/10/100 四种尺度的随机历史。
ONNX 最大绝对误差为 `0.0003662109375`，全部满足
`atol=1e-4, rtol=1e-4`。此验证确认导出数值一致性，未进行实机运行验证。

## 重新导出

在 `/home/xjtx/rl/HIMLoco` 执行：

```bash
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate gym4
# 本次依赖已装在 /tmp；临时目录被清理后可重新执行此命令。
python3 -m pip install --target /tmp/minich_onnx_deps --no-cache-dir --no-deps \
  onnx==1.17.0 onnxruntime==1.19.2 flatbuffers==24.3.25
PYTHONPATH=/tmp/minich_onnx_deps python3 legged_gym/legged_gym/scripts/export_minich_onnx.py \
  --run-dir legged_gym/logs/rough_minich/Oct12_09-14-06_smoke_minich \
  --checkpoint 1000
```

脚本需要对应的 `exported/policies/model_1000.jit`，在导出前核对其参数与 checkpoint。
