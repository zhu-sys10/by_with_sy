# VS Code 开发环境使用说明

> 目标：**不再打开 Keil 界面**。编译、烧录、调试、看波形全部在 VS Code 里完成。

## 文件布局

```
<工作区根>\
├─ .vscode\
│   ├─ tasks.json               编译 / 烧录 / 看波形的任务
│   ├─ launch.json              F5 调试配置（Cortex-Debug + J-Link）
│   ├─ c_cpp_properties.json    IntelliSense 头文件路径
│   └─ settings.json            Cortex-Debug 工具路径等
├─ tools\
│   └─ keil-build.ps1           真正干活的编译脚本（也可单独在终端跑）
├─ sentry\                      STM32 工程
│   └─ MDK-ARM\
│       ├─ sentry.uvprojx       ★ 唯一事实来源（加文件/改宏都改它）
│       ├─ flash.jlink          J-Link 烧录脚本
│       └─ standard\sentry.axf  编译产物（调试用的就是它）
└─ VS Code 使用说明.md           本文件
```

> ⚠️ `sentry\.vscode\` 里还有一份旧的 `settings.json`，那是以前单独打开 `sentry` 文件夹时留下的**孤儿配置，VS Code 不会读它**。真正生效的是上面这个 `.vscode\`。

---

## 0. 先做一次：重载窗口

新增的 `tasks.json` / `launch.json` 要重载才会生效：

> `Ctrl+Shift+P` → 输入 `Developer: Reload Window` → 回车

---

## 1. 环境里有什么（已经验证过的）

| 工具 | 路径 | 用途 |
|---|---|---|
| Keil 命令行 | `C:\Keil_v5\UV4\UV4.exe` | **编译**（只调命令行，不开界面） |
| ARMCC 编译器 | `C:\Keil_v5\ARM\ARMCC\Bin` | V5.06 update 7 (build 960) |
| J-Link | `D:\SPR\JLink_V896\JLink.exe` | 烧录 |
| J-Link GDB Server | `D:\SPR\JLink_V896\JLinkGDBServerCL.exe` | 调试 |
| RTT Viewer | `D:\SPR\JLink_V896\JLinkRTTViewer.exe` | 实时看 printf |
| J-Scope | `D:\SPR\JLink_V896\JScope.exe` | **实时波形** |
| arm-none-eabi-gdb | `D:\SPR\msys64\mingw64\bin\arm-none-eabi-gdb.exe` | Cortex-Debug 用 |
| MCU | **STM32F407IGHx** | J-Link 设备名填 `STM32F407IG` |

---

## 2. 编译

| 操作 | 方式 |
|---|---|
| 增量编译 | **`Ctrl+Shift+B`** |
| 全量重编（改了 `.h` / 宏） | `Ctrl+Shift+P` → `Tasks: Run Task` → **Keil: 全量重编** |
| 清理中间文件 | `Tasks: Run Task` → **Keil: 清理** |

编译报错会直接出现在 **Problems 面板**（`Ctrl+Shift+M`），点一下跳到行。

实测增量编译约 **2 秒**。

### 底层实际发生了什么

任务只是调 `tools\keil-build.ps1`，脚本里做的事：

```powershell
UV4.exe -b <proj>.uvprojx -j0 -o build.log   # build / -r 重编 / -c 清理
Get-Content build.log                        # UV4 只写日志文件，不写 stdout
```

也可以自己在终端里跑：

```powershell
powershell -ExecutionPolicy Bypass -File tools\keil-build.ps1 -Mode rebuild
```

UV4 退出码：`0` 无错无警 / `1` 有警告 / `2` 有错 / `3` 致命。

---

## 3. 烧录

### 方式 A：烧完就跑（不进调试）

`Ctrl+Shift+P` → `Tasks: Run Task` → **J-Link: 烧录 HEX**

会先自动编译，然后用 J-Link Commander 烧 `.hex` 并复位运行。

### 方式 B：烧录并进入调试

按 **`F5`**（或左侧「运行和调试」→ 选 **`J-Link: 烧录并调试`** → 绿三角）

---

## 4. 调试

按 `F5` 启动后：

| 功能 | 位置 |
|---|---|
| 变量 | 左侧「变量」面板 |
| **加监视** | 选中代码里的变量 → 右键 → **Add to Watch**，或直接在「监视」里手输 |
| 断点 | 行号左边点一下 |
| 调用栈 / 寄存器 | 左侧面板 |

### ★ Live Watch（实时监视，这个比 Keil 好用）

调试时打开命令面板 → **`Cortex Debug: Add to Live Watch`**，输入表达式，比如：

```
g6020.ctrl.e_last
g6020.ctrl.ws_last
g6020.ctrl.z2
g6020.ctrl.u_last
g6020.given_current
dm_4310.vel
INS_data.wz
```

**它会在程序全速运行的时候持续刷新**，不用暂停。适合盯住一个数看它抖不抖。

> ⚠️ Live Watch 靠 J-Link 后台读内存，刷新率不高（几十 Hz），**看不了高频抖震的波形** —— 那个要往下看第 5 节。

---

## 5. 看波形（抖震、振荡排查的关键）

### 5.0 ★ 推荐：用 Ozone（最省事）

`Ctrl+Shift+P` → `Tasks: Run Task` → **`Ozone: 打开工程（烧录+调试+波形）`**

或者直接双击工作区根目录的 **`sentry.jdebug`**。

工程已经配好了（设备 `STM32F407IG`、SWD 4 MHz、SVD 寄存器视图、ELF 路径），**打开就能用**。

| 功能 | 在 Ozone 里怎么找 |
|---|---|
| 源码 / 断点 / 单步 | 主界面，和 Keil 差不多 |
| 看变量 | `View → Watch`，直接输 `g6020.ctrl.ws_f` |
| 看寄存器 | `View → Register`（已加载 STM32F407IG 的 SVD） |
| **看波形** | **`View → Timeline`** |

> ⚠️ Ozone **V3.20 起把 “Data Graph” 窗口并入了 “Timeline”**，找不到 Data Graph 是正常的。
> 在 Timeline 里添加采样表达式（比如 `g6020.ctrl.ws_f`、`dm_4310.vel`）就能画随时间的变化。

> ⚠️ `sentry.jdebug` 里的 `AfterTargetDownload/AfterTargetReset` 是**专门为 Keil 编译的 ELF 加的，不要删**。
> 原因：Ozone 默认用 `Elf.GetEntryPointPC()` 取入口点，对 Keil/IAR 返回的是 C 运行库入口 `__main`，
> **会跳过 `startup` 里的 `SystemInit()`** → 主频只有 16 MHz → CAN 波特率、任务周期全乱。
> 改成从向量表第 1 个字（复位向量）取 PC 就对了。

**分工：**

| 干什么 | 用哪个 |
|---|---|
| 编译 | **VS Code**（`Ctrl+Shift+B`） |
| 烧录 / 调试 / 看波形 | **Ozone** |

改完代码先在 VS Code 编译，Ozone 会检测到 `.axf` 更新并提示重新载入。

---

### 5.1 J-Scope（备选）

`Ctrl+Shift+P` → `Tasks: Run Task` → **J-Link: J-Scope 波形**

J-Scope 里：

1. 选 **`HSS`** 模式（High Speed Sampling）
2. 载入 `sentry/MDK-ARM/standard/sentry.axf`（有符号表，能按名字选变量）
3. 勾选要看的变量 → Start

> 同样可以选 `RTT` 模式，但需要固件里有 RTT 代码（见 5.2）。

### 5.2 RTT（需要固件里加代码，目前还没加）

`Ctrl+Shift+P` → `Tasks: Run Task` → **J-Link: RTT Viewer**

⚠️ **现在按了没反应是正常的** —— 固件里还没有 RTT 代码。要用的话需要：
1. 往工程里加 `SEGGER_RTT.c/h`（J-Link 安装目录里没有带，需要另外下载或自己实现）
2. 在 Keil 工程里把它加进编译
3. 代码里调 `SEGGER_RTT_printf(0, "ws=%.3f\n", ws)`

**需要的话跟我说，我来接。**

---

## 6. 为什么还依赖 Keil 的编译器

工程里有两个 **armcc 编译出来的 `.lib`**，GCC 无法链接：

| 文件 | 大小 | 有没有源码 |
|---|---|---|
| `components/algorithm/AHRS.lib` | 34 KB | ❌ **没有** |
| `components/algorithm/arm_cortexM4lf_math.lib` | 12.7 MB | ✅ CMSIS-DSP，官网有源码 |

`AHRS_init()` / `AHRS_update()` 在 `Task/INS_task.c` 里被调用，所以 **AHRS.lib 不能用 GCC 替代**。

**⇒ 结论：编译仍走 Keil 命令行，但完全不用开 Keil 界面。**

> 如果哪天想彻底摆脱 Keil，必须先解决 `AHRS.lib`（用源码重写一个 Madgwick/Mahony 四元数 AHRS）。这是个独立工程，风险是姿态解算结果会变。

---

## 7. 出问题怎么排查

| 现象 | 原因 / 处理 |
|---|---|
| `Ctrl+Shift+B` 没反应 | 没重载窗口 → `Developer: Reload Window` |
| 任务报「**字符串缺少终止符**」 | 有人把 `tasks.json` 里的 `"type": "process"` 改成了 `"type": "shell"`。shell 模式会把整条命令再套一层单引号，而 PowerShell 命令里的单引号会把外层的提前闭合。**必须保持 `process`。** |
| 任务里中文显示乱码 | 用 `type: process` 跑 `.ps1` 就不会；旧版内联命令才会 |
| F5 报「找不到 JLinkGDBServerCL」 | 检查 `settings.json` 里的 `cortex-debug.JLinkGDBServerPath` |
| F5 报 gdb 找不到 | 检查 `D:\SPR\msys64\mingw64\bin\arm-none-eabi-gdb.exe` 是否存在 |
| 调试时变量显示 `<optimized out>` | 当前是 `-O1`（优化尺寸）。要在 Keil 工程里把 C 优化改成 `Level 0`（`Options for Target → C/C++ → Optimization`），再重编 |
| 断点打不上 / 乱跳 | 同上，优化等级太高 |
| 烧录报 `Cannot connect` | ① 检查 SWD 接线 ② 把 `flash.jlink` 里的 `speed 4000` 降到 `1000` |
| 烧录报找不到器件 | 检查 `flash.jlink` 里的 `device STM32F407IG` |
| J-Scope 连不上 | 本机 J-Link 是 **克隆版**（SN 123456），HSS 模式可能不支持 → 改用 Live Watch，或加 RTT |

---

## 8. 和 Keil 的关系

- **Keil 工程文件 `MDK-ARM/sentry.uvprojx` 仍然是唯一事实来源** —— 加/删源文件、改宏、改优化等级，都改它。
- VS Code 只是**调用它的命令行**。
- 所以：**新增 `.c` 文件时，还是要在 Keil 里加一次**（或者让我帮你改 `.uvprojx`）。

---

## 9. 常用快捷键速查

| 快捷键 | 作用 |
|---|---|
| `Ctrl+Shift+B` | 增量编译 |
| `F5` | 烧录 + 调试 |
| `Shift+F5` | 停止调试 |
| `F9` | 加/取消断点 |
| `F10` / `F11` | 单步跳过 / 单步进入 |
| `Ctrl+Shift+M` | 打开 Problems（看编译错误） |
| `Ctrl+Shift+P` | 命令面板（Live Watch、Run Task 都在这） |
| `Ctrl+Shift+V` | Markdown 预览（看 `小yaw控制器推导.md`） |
