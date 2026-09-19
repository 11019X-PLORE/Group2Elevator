# ElevatorDemo

## English

### Overview

ElevatorDemo is a WPILib 2026 Java project for team 11019 X.PLORE. It drives a dual-motor elevator on
Kraken X60 motors with TalonFX integrated encoders, software travel limits, AdvantageKit logging,
desktop simulation, and AdvantageScope telemetry — the control pattern ArmDemo validated, reduced to
the elevator alone and decoupled from the arm interlock so the elevator can be brought up by itself.

Motion is open-loop voltage. A single controller button press starts each movement, and the movement
continues until the corresponding software limit is reached. Presets that stop at a chosen height
instead of at a limit run through a proportional position controller.

**What this project is for.** It is a bring-up and validation rig: the code you flash onto an elevator
before a robot exists around it, to answer "does it travel the right distance, stop where it should,
and how long does each action take". It is deliberately not competition code — no autonomous, no
end effector, no vision, and no closed-loop trajectory tracking.

**What it is not.** The only position protection here is computed in the robot loop. That is enough
for a bench mechanism, and it is not the same claim as "hardware protected". §"What the limits
protect against" and [docs/CALIBRATION.md](docs/CALIBRATION.md) draw the line precisely.

### Positioning

| | ElevatorDemo | ArmDemo |
| --- | --- | --- |
| Mechanisms | elevator only | arm + elevator, with an interlock between them |
| Calibration | per-motor travels, or one shared value on a symmetric rig, plus a live mismatch diagnostic | two per-motor elevator travels, plus one arm scalar |
| Where limits are evaluated | inside `periodic()`, against that cycle's poll | in the command's `execute()`, one cycle behind |
| Untrusted sensors | stops the mechanism | not modelled |
| Wrong sign constants | latched direction fault | not modelled |
| Desktop simulation | yes (`ElevatorIOSim`) | no |
| Tests | 42, on a manual clock so nothing sleeps | 53 across two mechanisms |

### Safety Warning

> **Before every roboRIO power-on, manually place the elevator at its physical zero position (fully
> down).** After a full power cycle the TalonFX integrated encoders read zero on their own, so an
> elevator that is not at its physical zero position at power-on will make the software limits
> inaccurate and may damage the mechanism.
>
> Robot-code restarts (redeploy or soft restart) do **not** re-zero the encoders: the TalonFX
> positions survive a code restart, so the software limits stay accurate even when the code is
> restarted mid-match with the elevator away from zero.

Keep the robot disabled while positioning the elevator by hand. Stay clear of the elevator whenever
the robot is enabled.

### What the limits protect against

In place: limit checks run against the freshest sensor snapshot of the cycle, immediately before the
output is written, so a limit that triggers stops the mechanism in that same cycle; positions are
latency-compensated; a drive that stops answering, a failed status transaction, or a reading older
than the sensor budget stops the mechanism; a mistyped dashboard value falls back to its validated
default rather than silently disabling a ceiling; and a mechanism that demonstrably moves against its
command latches a fault instead of driving away from its stops.

Not in place: no hardware limit switch, no homing routine, no closed-loop tracking on the limit-style
moves, and no drive-layer soft limits — those exist in Phoenix 6 but three of their semantics could
not be confirmed off-hardware, and a second safety layer that silently does nothing is worse than
none. [docs/CALIBRATION.md §3](docs/CALIBRATION.md) lists exactly what to verify on the bench.

### Calibration

Fill in each motor's **travel**: the direction-corrected rotor rotations between the calibrated zero
and full extension, as positive magnitudes.

- **Symmetric rig** — both motors cover the same distance: fill `MAX_EXTENSION_ROTATIONS`, leave the
  two per-motor entries at `Double.NaN`.
- **Geared rig** — they do not (ArmDemo's bench elevator measured 3.4 and 14.5 rotations): fill
  `MOTOR_1_MAX_EXTENSION_ROTATIONS` and `MOTOR_2_MAX_EXTENSION_ROTATIONS`. One averaged number on
  such a rig stops the elevator at 62% of its stroke.

Then check `Elevator/CalibrationSuspect` in the log after the measurement: it compares the two live
readings and latches when a single shared value is being used to describe a rig whose motors travel
unequal distances. Which way the encoder turns and which polarity extends the mechanism are separate
constants (`EXTENSION_ROTOR_SIGN`, `EXTENSION_VOLTAGE_SIGN`), not the sign of the calibration value.

Until a usable travel exists, the subsystem reports uncalibrated and every movement refuses to run —
nothing on this mechanism moves on an unfilled or mistyped constant.

### Hardware Configuration

| Motor | CAN ID | Neutral Mode | Raw encoder while extending |
| --- | ---: | --- | --- |
| Kraken X60 / TalonFX 1 | 11 | Brake | Falls; `EXTENSION_ROTOR_SIGN = -1` corrects it |
| Kraken X60 / TalonFX 2 | 12 | Brake | Falls; same correction, mirrored voltage |

The driver controller is connected to USB port 0. Although the code uses `CommandPS5Controller`, this
controller reports Xbox-style raw button IDs in Driver Station.

### Controller Bindings

| Raw button | Action | Command behavior |
| ---: | --- | --- |
| 3 | Extend elevator | Runs until either motor reaches its own upper travel |
| 1 | Retract elevator | Runs until either motor falls back to the calibrated zero |
| 6 | Elevator to middle | Runs to `MID_EXTENSION_FRACTION` (default half) of each motor's own travel, from whichever side it is on |
| 7 | Re-zero the elevator | Accepted **only** while the mechanism rests on its lower stop; refused otherwise |
| 8 | Work preset | Position-controlled move to `WORK_EXTENSION_FRACTION` (default 0.75) of travel |

Buttons 1/3/6 are the ArmDemo-proven bindings, unchanged. Buttons 7/8 are new in this project and sit
on the shoulder buttons so they cannot collide with them; verify their raw IDs on your own pad in
Driver Station, since both commands are simply unreachable if a button never reports.

Every binding uses `onTrue`, so one press starts the command and the button does not need to be held.
Each movement records how long it ran; the duration is printed to the robot console and published as
`LastMoveDurationSeconds` / `LastMoveActionName`.

### Elevator Control

- Motors: CAN IDs 11 and 12, mirrored output (motor 2 always receives the negation of motor 1)
- Fixed movement voltage magnitude: `2V`
- Lower limit: the calibrated zero. Upper limit: each motor's own calibrated travel
- Either motor reaching a limit stops both motors, which is the same "whichever mechanism arrives
  first" rule ArmDemo used, now correct on a geared rig
- `stop()`, a limit trip, an interrupted command, and a refused movement all write zero volts

### Presets and position control

`moveToFractionCommand(0.0 … 1.0)` drives to a fraction of the calibrated travel and finishes inside
the arrive window; `holdAtFractionCommand(...)` makes the same move and keeps holding until something
interrupts it. Both feed through the same ramp and speed ceilings, are clamped so a preset can never
push past a hard limit, and are available for teleop or autonomous wiring — button 8 is one call of
`moveToFractionCommand`.

The controller is proportional, in fraction space, so it needs no stage ratio and inherits the
per-motor normalization. Its gravity feedforward **defaults to 0 V on purpose**: this rig's holding
voltage has not been measured, and a guessed number is worse than a documented blank. Against the
mock mechanism in the tests, the consequence is visible and reproducible — without feedforward the
loop parks about 0.05 of travel short of its target, and with a matched feedforward it holds on it.

### Speed, acceleration and sensor ceilings

All live-tunable, all re-read once per cycle, all sanitized: a non-finite or negative entry falls
back to the default below rather than silently turning a safety limit off, and `0` remains the
deliberate "this one is off" switch.

| `/SmartDashboard/` entry | Default | Effect |
| --- | ---: | --- |
| `Elevator Max Travel (/s)` | 1.5 | Cut the movement voltage when the faster motor is already at that many full travels per second |
| `Elevator Voltage Slew (V/s)` | 60 | Ramp the signed motor-1 voltage at most this fast, so reversals pass through zero |
| `Elevator Sensor Timeout (ms)` | 100 | Treat a position reading older than this as untrusted (0 disables just the age test) |
| `Elevator Position kP (V/travel)` | 2.0 | Preset controller gain — not bench-validated |
| `Elevator Arrive Tolerance (travel)` | 0.02 | How close a preset has to get to count as arrived — not bench-validated |
| `Elevator Gravity Feedforward (V)` | 0 | Voltage held at the target to carry the load — measure before raising |

### Data Flow (AdvantageKit IO pattern)

One pattern: **hardware → IO → subsystem → hardware**.

- `ElevatorIO` defines the contract: one `updateInputs(inputs)` poll per cycle filling an `@AutoLog`
  container (positions, applied voltages, whether each drive is answering, and how old each reading
  is), plus the output setters `setMotorVoltages` and `zeroEncoders`.
- `ElevatorIOTalonFX` is the real-hardware implementation. All Phoenix 6 detail lives there: Brake
  neutral mode, one `BaseStatusSignal.refreshAll` transaction per cycle, 100 Hz position/velocity
  signals, latency-compensated positions, and health reporting.
- `ElevatorIOSim` is a mock mechanism behind the same interface, so the loop runs on a laptop. It is a
  stand-in with a first-order response and a gravity threshold, not an identified model.
- `ElevatorSubsystem.periodic()` runs once per 20 ms loop and does five things in order: poll the
  tunables (sanitized once, shared by the whole cycle), poll the sensors and log them, update the
  motion estimate, turn the requested `MotionMode` into volts against the limits, and log outputs.
  **Commands never write to the hardware** — they request a mode and read back whether it completed.
- `ElevatorCalibration` resolves the constants into per-motor travels and exposes the ratio
  diagnostic. `RobotContainer` is wiring only: which IO implementation, which calibration, which
  buttons. `FakeElevatorIO` in the tests implements the same interface with plain fields.

### Simulation

```sh
./gradlew simulateJava
```

On the laptop `RobotBase.isReal()` is false, so `RobotContainer` builds `ElevatorIOSim` instead of
touching CAN, and the same limit, preset, ramp and fault logic runs against it. Advantages: a change
to the control code can be watched in AdvantageScope without a robot, and the mock stage can be
deliberately misconfigured — pass different per-motor travels to its constructor to reproduce a
geared rig. The physics constants in `ElevatorIOSim` were chosen to feel like a bench elevator at 2 V
and must not be read as measurements of the real mechanism.

### AdvantageScope Telemetry

Published under `RealOutputs/Elevator`:

- Positions: `Motor1ExtensionRotations`, `Motor2ExtensionRotations`, `ExtensionRotations`,
  `RawMotor1Rotations`, `RawMotor2Rotations`, `ExtensionFraction`
- Calibration: `Calibrated`, `MaxExtensionRotations` (motor 1), `Motor2MaxExtensionRotations`,
  `CalibrationSource`, `ConfiguredTravelRatio`, `MeasuredTravelRatio`, `CalibrationSuspect`
- Health: `SensorsTrusted`, `Motor1PositionAgeMilliseconds`, `Motor2PositionAgeMilliseconds`
  (`-1` = that drive never reported), `DirectionReversed`, `MotionDetected`
- Output and control: `Motor1AppliedVoltage`, `Motor2AppliedVoltage`, `Mode`,
  `TargetExtensionFraction`, `PositionErrorFraction`, `PositionArrived`, `RezeroAccepted`
- Motion: `Motor1TravelFractionPerSecond`, `Motor2TravelFractionPerSecond`
- Limits and timing: `AtLowerLimit`, `AtUpperLimit`, `LastMoveDurationSeconds`, `LastMoveActionName`

### Build, Test, and Deploy

Requirements: WPILib 2026, the Java 17 that ships with it, and the Phoenix 6 and AdvantageKit
vendordeps in `vendordeps/`.

```sh
./gradlew build          # compile, test, jar
./gradlew test           # 42 unit tests, no hardware
./gradlew simulateJava   # desktop loop against ElevatorIOSim
./gradlew deploy         # to the roboRIO
```

`.github/workflows/test.yml` runs `./gradlew test` on every push and pull request.

### Main Project Structure

```text
src/main/java/frc/robot/
├── Constants.java              # values only, including the calibration entries
├── ElevatorCalibration.java    # resolves them into per-motor travels + diagnostics
├── Main.java
├── Robot.java                  # LoggedRobot, receivers, hardware-vs-sim choice
├── RobotContainer.java         # wiring: IO implementation, calibration, bindings
└── subsystems/
    ├── ElevatorIO.java         # contract + @AutoLog inputs
    ├── ElevatorIOTalonFX.java  # real drives: latency compensation + health
    ├── ElevatorIOSim.java      # mock mechanism for desktop and headless runs
    └── ElevatorSubsystem.java  # modes, limits, ceilings, presets, faults
```

Tests are in `src/test/java/`, the calibration and limit reasoning is in
[docs/CALIBRATION.md](docs/CALIBRATION.md), and daily bilingual engineering logs are in `工程日志/`.

### Lineage

Ported from ArmDemo (`clone-projects/ArmDemo`): the IO layer, the telemetry key style, the
either-motor-stops-both rule, the latched mid-move direction, the signed-voltage ramp, the normalized
speed ceiling, the sanitized dashboard reads, and the deliberate absence of a startup re-zero. Four
things were changed on the way, each with a test behind it: the elevator's two per-motor travels
became the coordinate again (they had been collapsed into one averaged value), limit evaluation moved
from commands into `periodic()`, sensor trust became an input instead of an assumption, and the
sign conventions became two explicit constants with a fault that catches a wrong one.

## 中文

### 项目简介

ElevatorDemo 是 11019 X.PLORE 的 WPILib 2026 Java 项目。它用 Kraken X60 电机和 TalonFX 内置编码器
驱动双电机电梯，包含软件行程限位、AdvantageKit 日志、桌面仿真和 AdvantageScope 遥测——沿用 ArmDemo
验证过的控制模式，只保留电梯，并解除与机械臂的联锁，让电梯可以单独上机调试。

运动是开环电压。按一次手柄按键即启动动作，命令持续运行直到到达对应的软件限位；需要在指定高度停住
的预设位置命令则走比例位置控制器。

**这个项目用来做什么。** 它是上机前的调试与验证工装：机器人还没造出来时，先把这份代码烧到电梯上，
回答"行程对不对、该停的地方停不停、每个动作花多少时间"。它刻意不是比赛代码——没有自动阶段、没有
末端执行器、没有视觉、也没有闭环轨迹跟踪。

**它不是什么。** 这里唯一的位置保护运行在机器人代码循环里。对台架机构够用，但这不等于"硬件已受
保护"。下面"限位保护的范围"与 [docs/CALIBRATION.md](docs/CALIBRATION.md) 把这条线画得很清楚。

### 定位对比

| | ElevatorDemo | ArmDemo |
| --- | --- | --- |
| 机构 | 只有电梯 | 机械臂 + 电梯，并有两者之间的联锁 |
| 标定 | 分电机行程；对称机构可只填一个共享值；带实时不对称诊断 | 电梯两个分电机行程 + 机械臂一个标量 |
| 限位判断位置 | `periodic()` 内，用本周期的轮询结果 | 命令 `execute()` 内，慢一个周期 |
| 传感器不可信 | 立即停止机构 | 未建模 |
| 方向符号填反 | 锁存方向故障 | 未建模 |
| 桌面仿真 | 有（`ElevatorIOSim`） | 无 |
| 测试 | 42 个，跑在手动时钟上，不需要 sleep | 两个机构共 53 个 |

### 安全警告

> **每次 roboRIO 上电前，必须人工把电梯放到物理零位（完全降下）。** 完全断电重启后 TalonFX 内置
> 编码器会自行从零读数；如果上电时机构不在物理零位，软件限位会不准，并可能损坏机构。
>
> 重启机器人程序（重新部署或软重启）**不会**重新归零：TalonFX 位置在代码重启后保持不变，因此比赛中
> 途软重启时，即使电梯不在零位，软件限位依然准确。

人工调整机构时必须保持 Disabled；Enabled 后所有人员都应远离电梯的运动范围。

### 限位保护的范围

已具备：限位判断紧跟本周期传感器轮询、并在写输出之前执行，因此本周期看到就越限本周期停；位置做了
延迟补偿；电机掉线、状态事务失败、读数超过新鲜度预算都会使机构停止；Dashboard 数值误输入会回退到
已验证默认值而不是静默关掉安全上限；机构确实朝指令反方向走时锁存故障并停止一切运动。

未具备：没有硬件限位开关、没有归零流程、限位式动作不做闭环跟踪，也**没有启用驱动器层软限位**——
Phoenix 6 有这个 API，但其中三点语义在台架之外无法确认，而"静默失效的第二层保护"比没有保护更危险。
上机需要验证什么，见 [docs/CALIBRATION.md 第 3 节](docs/CALIBRATION.md)。

### 标定

填写每台电机的**行程**：从标定零位到完全伸出之间的转子圈数（方向修正后，取正数）。

- **对称机构**（两电机走同样距离）：填 `MAX_EXTENSION_ROTATIONS`，两个分电机值保持 `Double.NaN`。
- **变速比机构**（不一样；ArmDemo 台架实测 3.4 与 14.5 圈）：填
  `MOTOR_1_MAX_EXTENSION_ROTATIONS` 与 `MOTOR_2_MAX_EXTENSION_ROTATIONS`。这种机构用平均值做唯一
  限位，会在 **62%** 行程处就停住。

标定完成后在日志里检查 `Elevator/CalibrationSuspect`：它比较两路实时读数，当"只用一个共享值"却存在
行程差时会置真。编码器绕向与电压方向是两个独立常量（`EXTENSION_ROTOR_SIGN`、
`EXTENSION_VOLTAGE_SIGN`），不再靠标定值的正负号表达。

在填出可用行程之前，子系统报告"未标定"，所有运动命令拒绝运行——没填或误输入的常量驱动不了机构。

### 硬件配置

| 电机 | CAN ID | 停止模式 | 伸出时的原始编码器读数 |
| --- | ---: | --- | --- |
| Kraken X60 / TalonFX 1 | 11 | Brake | 减小，由 `EXTENSION_ROTOR_SIGN = -1` 修正 |
| Kraken X60 / TalonFX 2 | 12 | Brake | 减小，同一修正，电压镜像 |

驾驶员手柄接 USB 端口 0。代码使用 `CommandPS5Controller`，但当前手柄在 Driver Station 中返回 Xbox
风格的原始按键编号。

### 手柄按键

| 原始按键 | 动作 | 命令行为 |
| ---: | --- | --- |
| 3 | 电梯伸出 | 运行到任一电机到达各自上限 |
| 1 | 电梯收回 | 运行到任一电机回到标定零位 |
| 6 | 电梯中位 | 运行到各电机自身行程的 `MID_EXTENSION_FRACTION`（默认一半），从任意一侧靠近 |
| 7 | 重新归零 | **仅**在机构停在下限位时接受，其他情况拒绝 |
| 8 | 工作高度预设 | 位置控制运行到行程的 `WORK_EXTENSION_FRACTION`（默认 0.75） |

按键 1/3/6 是 ArmDemo 验证过的映射，未改动；按键 7/8 是本项目新增，刻意放在两个肩键上避免冲突。
上机前请在 Driver Station 里确认这两个原始编号——如果按键本身不触发，这两条命令只是不可达，不会有
其他副作用。

所有按键都用 `onTrue`，按一次即启动，不需要按住。每个运动命令记录自身运行时长：完成后打印到机器人
控制台，并作为 `LastMoveDurationSeconds` / `LastMoveActionName` 发布。

### 电梯控制

- 电机 CAN ID 11 与 12，输出镜像（电机 2 永远拿电机 1 的相反数）
- 固定运动电压 `2V`
- 下限：标定零位；上限：各电机自己的标定行程
- 任一电机触限即两台停止——与 ArmDemo 相同的"任一机构先到即停"规则，但在变速比机构上终于是对的
- `stop()`、限位触发、命令被打断、被拒绝的运动，四种情况都写零伏

### 预设与位置控制

`moveToFractionCommand(0.0 … 1.0)` 运行到标定行程的某个分数，进入到位窗口后结束；
`holdAtFractionCommand(...)` 同样运行但一直握住，直到被打断。两者都经过同一个电压斜坡与速度上限，
都被夹紧到"预设不可能推过硬限位"，并且都可直接供 teleop / 自动阶段调用——按键 8 就是一句
`moveToFractionCommand`。

控制器是纯比例、工作在行程分数空间，因此不需要级间速比，并自动继承分电机归一化。它的重力前馈
**默认 0V 是刻意的**：这台机构的支持电压还没测过，猜一个数比留一个空位更糟。在测试用的模拟机构上
这个后果清晰可复现——不加前馈时回路会停在目标下方约 0.05 行程处，配上匹配的前馈就能停在目标上。

### 速度、加速度与传感器上限

全部可在 NetworkTables 在线调节，每周期各读一次并统一消毒：非有限或负值的误输入回退到下表默认值，
而不是静默关闭安全上限；`0` 仍然是"这一项主动关闭"的开关。

| `/SmartDashboard/` 下的项 | 默认值 | 作用 |
| --- | ---: | --- |
| `Elevator Max Travel (/s)` | 1.5 | 较快电机已达该"每秒全行程数"时把运动电压归零 |
| `Elevator Voltage Slew (V/s)` | 60 | 带符号的电机 1 电压每秒最多变化这么多，换向经过零点 |
| `Elevator Sensor Timeout (ms)` | 100 | 位置读数老于此值即判为不可信（设 0 只关闭年龄这一项） |
| `Elevator Position kP (V/travel)` | 2.0 | 预设控制器增益——未经台架验证 |
| `Elevator Arrive Tolerance (travel)` | 0.02 | 预设算作"到位"的窗口——未经台架验证 |
| `Elevator Gravity Feedforward (V)` | 0 | 停在目标上用于承担重力所需的电压——先测再调 |

### 数据流（AdvantageKit IO 模式）

同一条数据流：**硬件 → IO → 子系统 → 硬件**。

- `ElevatorIO` 定义契约：每周期一次的 `updateInputs(inputs)`，填入 `@AutoLog` 容器（位置、输出
  电压、每台驱动器是否在应答、读数多旧），加上输出方法 `setMotorVoltages`、`zeroEncoders`。
- `ElevatorIOTalonFX` 是真机实现，所有 Phoenix 6 细节锁在这里：Brake 停止模式、每周期一次
  `BaseStatusSignal.refreshAll` 批量事务、100 Hz 位置/速度信号、延迟补偿、健康度上报。
- `ElevatorIOSim` 用同一接口提供模拟机构，让整条回路能在笔记本上跑。它是带一阶响应与重力阈值的
  替身，不是辨识出来的模型。
- `ElevatorSubsystem.periodic()` 在 20 ms 循环里恰好执行一次，按顺序做五件事：轮询调参项（每周期
  消毒一次、全周期共享）、轮询传感器并记录、更新运动估计、把请求的 `MotionMode` 对照限位换成电压、
  记录输出。**命令永不直接写硬件**——它们只申请一个模式，并读回是否完成。
- `ElevatorCalibration` 把常量解析成分电机行程并给出比值诊断；`RobotContainer` 只做接线：哪个 IO
  实现、哪份标定、哪些按键；测试里的 `FakeElevatorIO` 用普通字段实现同一接口。

### 桌面仿真

```sh
./gradlew simulateJava
```

在笔记本上 `RobotBase.isReal()` 为假，`RobotContainer` 因此构造 `ElevatorIOSim` 而不碰 CAN，同一套
限位、预设、斜坡与故障逻辑照常在台架替身上运行。好处是：改控制代码不用接机器人就能在 AdvantageScope
里看；而且可以故意配错模拟机构——给它的构造函数传不同的分电机行程，就能复现变速比机构。
`ElevatorIOSim` 里的物理常数只是"像一台 2V 台架电梯"，不能当成对真实机构的测量。

### AdvantageScope 遥测

发布在 `RealOutputs/Elevator` 下：

- 位置：`Motor1ExtensionRotations`、`Motor2ExtensionRotations`、`ExtensionRotations`、
  `RawMotor1Rotations`、`RawMotor2Rotations`、`ExtensionFraction`
- 标定：`Calibrated`、`MaxExtensionRotations`（电机 1）、`Motor2MaxExtensionRotations`、
  `CalibrationSource`、`ConfiguredTravelRatio`、`MeasuredTravelRatio`、`CalibrationSuspect`
- 健康：`SensorsTrusted`、`Motor1PositionAgeMilliseconds`、`Motor2PositionAgeMilliseconds`
  （`-1` 表示该驱动器从未上报）、`DirectionReversed`、`MotionDetected`
- 输出与控制：`Motor1AppliedVoltage`、`Motor2AppliedVoltage`、`Mode`、`TargetExtensionFraction`、
  `PositionErrorFraction`、`PositionArrived`、`RezeroAccepted`
- 运动：`Motor1TravelFractionPerSecond`、`Motor2TravelFractionPerSecond`
- 限位与计时：`AtLowerLimit`、`AtUpperLimit`、`LastMoveDurationSeconds`、`LastMoveActionName`

### 构建、测试与部署

环境要求：WPILib 2026、其自带的 Java 17、`vendordeps/` 中的 Phoenix 6 与 AdvantageKit 依赖。

```sh
./gradlew build          # 编译 + 测试 + 打包
./gradlew test           # 42 个单元测试，不需要硬件
./gradlew simulateJava   # 桌面回路跑 ElevatorIOSim
./gradlew deploy         # 部署到 roboRIO
```

`.github/workflows/test.yml` 会在每次 push 与 PR 上运行 `./gradlew test`。

### 主要项目结构

```text
src/main/java/frc/robot/
├── Constants.java              # 只放数值，包括标定项
├── ElevatorCalibration.java    # 解析成分电机行程 + 诊断
├── Main.java
├── Robot.java                  # LoggedRobot、数据接收器、真机/仿真选择
├── RobotContainer.java         # 接线：IO 实现、标定、按键
└── subsystems/
    ├── ElevatorIO.java         # 契约 + @AutoLog 输入
    ├── ElevatorIOTalonFX.java  # 真机：延迟补偿 + 健康度
    ├── ElevatorIOSim.java      # 桌面与无硬件运行的模拟机构
    └── ElevatorSubsystem.java  # 模式、限位、上限、预设、故障
```

测试在 `src/test/java/`，标定与限位的设计理由在 [docs/CALIBRATION.md](docs/CALIBRATION.md)，每日双语
工程日志在 `工程日志/`。

### 与 ArmDemo 的关系

从 ArmDemo（`clone-projects/ArmDemo`）移植：IO 分层、遥测键风格、"任一电机到限即双停"、中位命令的
方向锁存、带符号电压斜坡、归一化速度上限、Dashboard 消毒读取、以及刻意不在启动时归零。移植过程中
改了四处，每一处都有测试兜底：电梯重新按各电机自身行程归一化（此前被压成一个平均值）、限位判断从
命令移进 `periodic()`、传感器可信度从假设变成输入、方向符号变成两个显式常量并配一个抓错的故障锁存。
