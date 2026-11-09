# 双 yaw 世界角瞄准 — 上下文交接文档

> 直接贴到新会话开头即可。所有结论截至 2026-10-19。

---

## 1. 项目位置

```
D:\SPR\SPR_code\old_sentry\sentry try_gimbal
```

- Keil 工程：`sentry\MDK-ARM\sentry.uvprojx`（用相对路径，零绝对路径）
- **注意：同目录下还有个 `super_sentry` 备份工程，不要打开它。** 曾经因为开错工程导致"改了代码没效果"，查了很久。
- 仓库里绝大部分代码是另一个不相关的项目，**只有下面这几个文件有用**。

---

## 2. 硬件

| 部位 | 型号 | 控制方式 |
|---|---|---|
| 大 yaw | 达妙 DM-J4310 | MIT 模式，位置环在电机内部 |
| 小 yaw | 大疆 GM6020 | 板载 ADRC/ESO 位置环（本仓库自己写的） |

- 两轴都绕**竖直轴**，平行、非同轴，间距 92 mm
- C 板装在小 yaw 上，**IMU 也在上面**
- 当前是**台架 DEMO**：没有底盘，大 yaw 底座固定在台面上
- 小 yaw **暂时不能上负载**

CAN：
- DM4310 控制帧 ID `0x05`，反馈帧 ID `0x55`，挂在 `hcan1`
- GM6020 也在 `hcan1`

---

## 3. 哪些文件重要

| 文件 | 作用 |
|---|---|
| `sentry/Task/dm_motor.c/.h` | DM4310 MIT 协议驱动 |
| `sentry/Task/gimbal_6020.c/.h` | GM6020 驱动 + ESO 控制器 |
| `sentry/Task/by_with_sy.c/.h` | 主任务：模式控制、目标分配、输出 |
| `sentry/Task/yaw_aim.c/.h` | 世界角瞄准算法（本次新写） |
| `sentry/Task/gimbal_task.c` | 旧参考工程，**只读**，用来参考 IMU 角度怎么用 |

---

## 4. 当前算法（核心）

### 4.1 坐标关系

底座固定，所以只有：

```
world = 大 yaw 轴角 + 小 yaw 角        （常数零点偏移会消掉）
```

**世界角以 IMU 为准**：`INS_data.angle_yaw`，上电位置为 0。
编码器只用来算各自相对位置，**不用来推算世界角**（用户明确要求）。

### 4.2 `yaw_aim_update` 全部逻辑

```c
err_world  = rad_format(yaw_aim_target_rad - INS_data.angle_yaw);
small_raw  = g6020.angle + err_world;
*small_cmd = clampf(small_raw, -1.00f, +1.00f);
big_target = yaw_big_angle + small_raw;

/* 大 yaw：摇杆优先，否则自动回中 */
if (big_manual_rate != 0.0f)
    s_big_ref += big_manual_rate * dt;
else
    s_big_ref += clampf(YAW_AIM_CENTER_GAIN * rad_format(big_target - s_big_ref),
                        -YAW_AIM_CENTER_SLEW, +YAW_AIM_CENTER_SLEW) * dt;

s_big_ref = clampf(s_big_ref, -YAW_BIG_MAX_RAD, +YAW_BIG_MAX_RAD);
*big_cmd  = s_big_ref;
```

### 4.3 为什么这样写

**小 yaw 目标 = 当前角 + 世界误差**，一行同时满足三个需求：

| 需求 | 机制 |
|---|---|
| 给新目标，小 yaw 迅速转过去 | 大 yaw 还没动，小 yaw 独吃全部误差 |
| 大 yaw 跟随，小 yaw 慢慢回中 | 大 yaw 一动，世界误差变小，`small_raw` 自动缩小 |
| 大 yaw 手动转动，世界角不变 | 世界误差自动反向，小 yaw 目标自动反向 |

**`big_target` 不会自激**：

```
big_target = big + small + (target − world)
world      = big + small + 常数
→ big_target = target − 常数
```

大 yaw 自己转 δ，`big` 涨 δ、`err_world` 掉 δ，互相抵消。所以 `big_target` 只随目标和底盘变化。

### 4.4 大 yaw 的目标含义

`big_target` = **"小 yaw 回到 0 时，大 yaw 应该停在哪"**。

---

## 5. 模式控制

左拨杆 `rc_ctrl.rc.s[1]`：

| 位置 | 模式 | 行为 |
|---|---|---|
| 上 | `YAW_MODE_AIM` | 世界角瞄准 |
| 中 | `YAW_MODE_MANUAL` | 摇杆直接积分目标角 |
| 下 | `YAW_MODE_DISARM` | 小 yaw 电流置 0（大 yaw 仍保持） |
| 遥控器丢信号 | 强制 `DISARM` | 安全兜底 |

摇杆通道：

| 通道 | 手动模式 | 瞄准模式 |
|---|---|---|
| `ch[2]` 左摇杆左右 | 大 yaw 目标角积分（11 rad/s 满杆） | **手动转大 yaw（4 rad/s 满杆）** |
| `ch[0]` 右摇杆左右 | 小 yaw 目标角积分（3.6 rad/s 满杆） | **微调世界目标角（3 rad/s 满杆）** |

**摇杆有死区 ±10，死区内返回 0；瞄准模式下 `big_manual_rate == 0` 时自动回中才生效。**

### 进入瞄准时会发生什么

```c
yaw_aim_enter(INS_data.angle_yaw, yaw_big_angle);
// → 目标 = 当前世界角，s_big_ref = 当前大 yaw 角
```

**防甩动设计，但也是测试陷阱：每次拨到瞄准，目标都会重置成当前角度。**

### 输出

```c
dm_motor_mit(&dm_4310, big_start + yaw_big_set / YAW_BIG_RATIO);
gimbal_6020_set_angle(&g6020, yaw_small_set, rate_rel);
// rate_rel = GIMBAL_6020_GYRO_TURN * INS_data.wz - YAW_BIG_RATIO * dm_4310.vel
```

---

## 6. 当前所有参数

### `by_with_sy.h`

```c
#define DM_4310_CAN_ID    (0x05u)
#define DM_4310_MASTER_ID (0x55u)
#define YAW_BIG_MAX_RAD   (6.2831853f)   /* ±2π */
#define YAW_SMALL_MAX_RAD (1.05f)        /* 物理 ±60° */
#define YAW_BIG_RATIO     (1.0f)         /* 电机输出轴 → yaw 轴传动系数 */
```

### `by_with_sy.c`

```c
#define YAW_BIG_RC_CH        (2)
#define YAW_SMALL_RC_CH      (0)
#define RC_CH_MAX            (660.0f)
#define RC_DEADBAND          (10)
#define YAW_CONTROL_DT       (0.001f)
#define YAW_BIG_MAN_RATE     (11.0f)   /* 手动大 yaw 满杆 rad/s */
#define YAW_SMALL_MAN_RATE   (3.6f)    /* 手动小 yaw 满杆 rad/s */
#define YAW_AIM_TRIM_RATE    (3.0f)    /* 瞄准微调满杆 rad/s */
#define YAW_AIM_BIG_MAN_RATE (4.0f)    /* 瞄准手动大 yaw 满杆 rad/s */
```

### `yaw_aim.h`

```c
#define YAW_AIM_SMALL_LIMIT  (1.00f)   /* 小 yaw 可用行程 rad */
#define YAW_AIM_CENTER_SLEW  (2.0f)    /* 大 yaw 自动回中最大速率 rad/s */
#define YAW_AIM_CENTER_GAIN  (1.5f)    /* 回中比例 1/s */
#define YAW_AIM_CMD_IDLE     (-999.0f) /* 调试口空闲值 */
```

### `dm_motor.h`

```c
#define DM_P_MIN (-12.5f)  #define DM_P_MAX (12.5f)
#define DM_V_MIN (-30.0f)  #define DM_V_MAX (30.0f)
#define DM_T_MIN (-10.0f)  #define DM_T_MAX (10.0f)
#define DM_KP_MIN (0.0f)   #define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)   #define DM_KD_MAX (5.0f)
#define DM_POS_KP_DEFAULT (15.0f)
#define DM_POS_KD_DEFAULT (1.0f)
#define DM_START_DELAY_MS (500u)
```

### `gimbal_6020.h`（已调好，**不要随意动**）

```c
#define GIMBAL_6020_B0       (0.14f)    /* 输入增益 counts → rad/s² */
#define GIMBAL_6020_WC       (45.0f)    /* 目标穿越频率 rad/s ≈ 7.2 Hz */
#define GIMBAL_6020_ZETA     (0.66f)
#define GIMBAL_6020_KI_INT   (0.0f)
#define GIMBAL_6020_ILIM     (3000.0f)
#define GIMBAL_6020_ESO_WO   (80.0f)
#define GIMBAL_6020_ESO_GAIN (1.0f)
#define GIMBAL_6020_COMP_LPF (6.0f)     /* 只滤补偿量，抑制 ~20Hz 机械共振 */
#define GIMBAL_6020_DLIM     (500.0f)
#define GIMBAL_6020_CLIM     (16000.0f)
#define GIMBAL_6020_CSLEW    (0.0f)
#define GIMBAL_6020_TSQ_PERIOD (300.0f)
#define GIMBAL_6020_TSQ_LIM  (6000.0f)
#define GIMBAL_6020_WLP      (90.0f)    /* 角速度低通 Hz */
#define GIMBAL_6020_WREF_LP  (40.0f)
#define GIMBAL_6020_WREF_LIM (6.0f)
#define GIMBAL_6020_REF_SLEW (6.0f)
#define GIMBAL_6020_OUT_TURN  (1.0f)
#define GIMBAL_6020_GYRO_TURN (1.0f)
#define GIMBAL_6020_BIG_TURN  (1.0f)    /* 已废弃，被 YAW_BIG_RATIO 取代 */
```

---

## 7. 小 yaw 控制器的已知结论（前期花了很久调出来的）

模型：`θ̈ = b0·u + d`，LESO 双极点 `−wo`。

- **b0 = 0.14 是实测标定值**。之前填的 0.005 小了 28 倍，导致满杆输出只有 270 counts。
  标定方法：开方波（`test_sq_amp`），速度三角波的斜率 = `b0 × 电流`。
  换负载必须重测 b0，其余参数不用动。
- **物理刚度 = `b0·kp = wc²`**，所以"手感硬不硬"只由 `wc` 决定。
- 原来的极点配置实际穿越频率 151 rad/s、相位裕度 −17°，这就是之前一直抖的原因。
- ESO 补偿 `eso_comp = 1` 后，静差从 ~0.05 rad 降到 0。
- **ESO 的 `wo = 80` 会激起 ~20 Hz 机械共振**（`comp = 0` 时不出现）。
  修法：只对补偿量加低通 `comp_lpf = 6 Hz`，不滤观测器本身。
- 曾经 `dist_limit = 22`（b0=0.005 时代的值）导致 ESO 饱和发散，已改成 500。
- 已知机械问题（没解决，与代码无关）：
  - 位置相关力矩 ±64 rad/s²（怀疑走线）
  - 静/动摩擦不对称
  - ~20 Hz 机械共振

---

## 8. 调试方法

### Ozone

- **显示格式必须改成 Floating point**，否则浮点会显示成 hex，而且输入会被当成十六进制解析（输入 `45` 变成 0x45=69）。
  右键那一行 → `Format` / `Display Format` → `Floating point`。
- 调试写入口：`yaw_aim_cmd_deg`（单位**度**）。写非 −999 生效一次，然后自动复位成 −999。

### Watch 变量

| 变量 | 含义 |
|---|---|
| `yaw_aim_cmd_deg` | 调试写入口，平时是 −999 |
| `yaw_aim_target_rad` | 当前世界目标（rad） |
| `yaw_world_angle` | **IMU 实测世界角** |
| `yaw_enc_angle` | 编码器推算世界角（`big_angle + g6020.angle`） |
| `yaw_big_angle` | 大 yaw 轴角 |
| `yaw_big_set` | 大 yaw 目标 |
| `yaw_small_set` | 小 yaw 目标 |
| `g6020.angle` | 小 yaw 实际相对角 |
| `dm_4310.pos` / `.vel` / `.err` | DM 电机反馈 |

**关键诊断：`yaw_world_angle − yaw_enc_angle` 应该是一个常数。**
- 恒定 → `YAW_BIG_RATIO` 和符号都对
- 线性漂移 → `YAW_BIG_RATIO` 不对
- 越来越大的 2 倍 → IMU/编码器符号反了

### Keil

- 新建的 `.c` 文件必须手动加进工程分组：
  右键 Project 窗口里的 `Task` 分组 → `Add Existing Files to Group 'Task'...`
- 头文件不用加。

---

## 9. 已确认 / 未确认

### 已确认（有 Ozone 数据）

命令 90° 时：

```
yaw_aim_cmd_deg    = -999        （已消费）
yaw_aim_target_rad = 1.57079637  （正好 90°）
yaw_big_set        = 1.57079637  （大 yaw 参考到位）
dm_4310.vel        = -0.0073     （停稳）
yaw_small_set      = 0.0174      （小 yaw 回中，差 1°）
dm_4310.err        = 0x01        （已使能）
dm_4310.kp=15  kd=1
```

**结论：瞄准逻辑本身是对的。** `yaw_small_set = 0.0174` 是 `kp = 15` 留下的稳态误差。

### 未确认

1. **`YAW_BIG_RATIO` 到底是 1.0 还是别的。**
   曾经报过"命令 180°，实际只转 90°"，至今没定位。
   可能是：(a) 有 2:1 减速；(b) 测量时其实是在量增量；(c) 中途重拨了拨杆导致目标被重置。
   **用 `yaw_world_angle − yaw_enc_angle` 判断，别再纠结量角器。**
2. **"大 yaw 转得太慢"**。可能原因：
   - `YAW_BIG_RATIO` 填错（轴上速度被砍半）
   - `DM_POS_KP_DEFAULT = 15` 太软。可以试 `kp = 40`、`kd = 2`
3. 场景 3（推左摇杆时世界角纹丝不动）**还没实测过**。

---

## 10. 待办

1. 编译、烧录、跑四步测试：
   - 拨到瞄准不动 → 两个电机都不动
   - 写 `yaw_aim_cmd_deg = 90` → 小 yaw 先冲、大 yaw 后追、小 yaw 回 0
   - **推左摇杆 → `yaw_world_angle` 必须钉死不动**（最重要）
   - 松摇杆 → 大 yaw 慢慢回、小 yaw 归零
2. 用 `yaw_world_angle` vs `yaw_enc_angle` 定 `YAW_BIG_RATIO`
3. 决定要不要加大 `DM_POS_KP_DEFAULT`
4. 可选：`DISARM` 時也放开大 yaw（目前仍保持）
5. 可选：上负载后重测 b0（20 分钟流程）
6. 可选：耦合前馈 `u_ff = θ̈_b / b0`
7. 可选：~20 Hz 机械共振的陷波滤波器

---

## 11. 踩过的坑（别再重复）

| 坑 | 现象 | 原因 |
|---|---|---|
| 开错工程 | 改了代码没效果 | 打开了 `super_sentry` 备份 |
| 文件没加进 Keil | `L6218E: Undefined symbol` | Keil 不扫文件夹，`.c` 必须手动加 |
| Ozone 显示 hex | 填角度填不进去 | 显示格式是 Hex，输入 `45` 被当成 0x45 |
| Ozone Watch 不刷新 | 数值自相矛盾（kp=3150 但 wc=38） | `Refresh: Off` |
| 每次拨到瞄标准备重置 | "给 180 只转 90" | `yaw_aim_enter` 把目标锁成当前角 |
| `rad_format` 编译报错 | `PI` 未定义 | `user_lib.h` 用了 PI 但没定义，需 `#include "arm_math.h"` |
| ESO 饱和 | `eso_rate` 发散（−8.6 而 `rate_last` = +1.38） | `dist_limit = 22` 太小，改 500 |
| ESO 激起共振 | ~20 Hz 机械共振 | 补偿量没有低通，加 `comp_lpf = 6` |

---

## 12. 用户偏好

- **中文交流**
- **代码注释要极度简洁**（原话："把和大小yaw有关的代码的注释简洁简洁简洁！！！！！"）
- **变量命名必须能看懂**，不要魔数（比如不要 `ch * 0.0000045f`，要 `stick * RATE * DT`）
- 需要解释**为什么**，不只要结论
- 信号与系统基础较弱：带宽、相位裕度、延时这些概念要展开讲
