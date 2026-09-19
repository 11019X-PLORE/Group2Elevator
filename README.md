# ElevatorDemo

## English

### Overview

ElevatorDemo is a WPILib 2026 Java project for team 11019 X.PLORE. It controls a dual-motor elevator with Kraken X60 motors, TalonFX integrated encoders, software travel limits, AdvantageKit logging, and AdvantageScope telemetry — the same control pattern ArmDemo validated, reduced to the elevator alone.

The elevator uses a fixed movement voltage. A single controller button press starts each movement command, and the command continues until the corresponding software limit is reached.

**One fill-in limit**: the only value that must be measured before the elevator runs is `Constants.Elevator.MAX_EXTENSION_ROTATIONS`. Until it holds a real number, the subsystem reports uncalibrated and every movement command refuses to run.

### Safety Warning

> **Before every roboRIO power-on, manually place the elevator at its physical zero position (fully down).** After a full power cycle the TalonFX integrated encoders read zero on their own, so an elevator that is not at its physical zero position at power-on will make the software limits inaccurate and may damage the mechanism.
>
> Robot-code restarts (redeploy or soft restart) do **not** re-zero the encoders: the TalonFX positions survive a code restart, so the software limits stay accurate even when the code is restarted mid-match with the elevator away from zero.

Keep the robot disabled while positioning the elevator by hand. Stay clear of the elevator whenever the robot is enabled.

### Measuring the One Limit (First-Use Calibration)

1. Deploy this code with `MAX_EXTENSION_ROTATIONS` still `0.0` — the elevator is safe: it will not move.
2. With the robot **disabled**, hand-raise the elevator from its physical zero to full extension and read `Elevator/ExtensionRotations` in AdvantageScope (the direction-corrected average of both TalonFX positions). A powered alternative: fill a generous overestimate (e.g. `999`) as a temporary limit, extend briefly at low risk of overtravel, disable, read the same value, then overwrite it — stay ready to hit disable, because the software limit is not protecting you yet.
3. Fill that number into `Constants.Elevator.MAX_EXTENSION_ROTATIONS` and redeploy. This is the whole calibration.

The sign of the filled value is the measured extension direction: if your rig winds the opposite way, the measurement comes out negative — fill it in as-is and every voltage sign follows automatically. A value of `0.0` (or any mistyped entry) keeps the elevator refused-to-move.

### Hardware Configuration

| Motor | CAN ID | Neutral Mode | Encoder Direction During Positive Extension |
| --- | ---: | --- | --- |
| Kraken X60 / TalonFX 1 | 11 | Brake | Raw value decreases; software negates it |
| Kraken X60 / TalonFX 2 | 12 | Brake | Raw value decreases; software negates it |

The driver controller is connected to USB port 0. Although the code uses `CommandPS5Controller`, this controller reports Xbox-style raw button IDs in Driver Station.

### Controller Bindings

| Raw Button | Action | Command Behavior |
| ---: | --- | --- |
| 3 | Extend elevator | Runs until either motor reaches the one upper limit |
| 1 | Retract elevator | Runs until either motor falls back to the zero-position lower limit |
| 6 | Elevator to middle | Runs until either motor reaches its share of the middle working extension (default half travel), from whichever side the elevator is on |

Each binding uses `onTrue`, so one press starts the command and the button does not need to remain held. Every movement command records how long it ran; the duration is printed to the robot console and published as telemetry (`LastMoveDurationSeconds` / `LastMoveActionName`).

### Elevator Control and Limits

- Motors: CAN IDs 11 and 12
- Fixed movement voltage magnitude: `2V`
- Extension output: motor 1 receives `+2V` (times the measured direction sign); motor 2 receives the negation
- Retraction output: the opposite signs
- Measurement: both TalonFX positions are direction-corrected (extension positive) and compared against the one shared limit
- Lower limit: the calibrated zero (`0` rotations)
- Upper limit: `MAX_EXTENSION_ROTATIONS` — the one fill-in value
- Middle working extension (button 6): `MID_EXTENSION_FRACTION` (default half) of the calibrated travel

If either motor reaches the upper or lower limit first, both motors stop. The mid command latches its direction once at start, so a skipped tolerance window cannot reverse it into an oscillation — the far hard limit ends the run instead.

**Calibration gate**: while `MAX_EXTENSION_ROTATIONS` is `0.0` (uncalibrated), `extend()`, `retract()`, `moveTowardMidExtension()`, and all three movement commands stop immediately with zero volts. This is what makes "fill one limit and use" safe: an unfilled constant can never drive the mechanism.

### Speed and Acceleration Limits

The validated open-loop voltage motion is kept; the spec's "settable max speed / max acceleration" is layered on top as live-tunable safety ceilings:

- **Acceleration limit**: movement voltages ramp at most a configurable number of volts per second (`Elevator Voltage Slew (V/s)`, default `60 V/s` — effectively instant at 2V). The ramp applies to the signed voltage, so a direction reversal passes through zero volts instead of snapping to the opposite polarity.
- **Speed limit**: when the measured motion already runs at the cap in the commanded direction, the movement voltage is cut to zero. Each motor's extension velocity is normalized by the one shared travel, and the faster motor is compared against `Elevator Max Travel (/s)` (default `1.5` full travels per second).
- Setting any entry to `0` disables that limit; a mistyped (non-numeric or negative) entry falls back to its validated default instead of silently disabling the limit.

### Action Timing

Every movement command (extend/retract/middle) measures its own runtime, prints `Elevator action <name> took <t> s` to the robot console, and publishes `LastMoveDurationSeconds` plus `LastMoveActionName` under `RealOutputs/Elevator` — the numbers for the spec's per-action completion-time records.

### AdvantageScope Telemetry

Elevator values are published under `RealOutputs/Elevator`, including:

- `Motor1ExtensionRotations` / `Motor2ExtensionRotations` (direction-corrected, per motor)
- `ExtensionRotations` (the averaged value to read when measuring the limit)
- `RawMotor1Rotations` / `RawMotor2Rotations`
- `ExtensionFraction` (average fraction of the calibrated travel)
- `Motor1AppliedVoltage` / `Motor2AppliedVoltage`
- `AtLowerLimit` / `AtUpperLimit` / `Calibrated`
- `MaxExtensionRotations` (the filled-in limit, for visibility)
- `Motor1TravelFractionPerSecond` / `Motor2TravelFractionPerSecond`
- `LastMoveDurationSeconds` / `LastMoveActionName`

### Data Flow (AdvantageKit IO Pattern)

One data-flow pattern: **hardware → IO → subsystem → hardware**.

- `ElevatorIO` defines the hardware contract: an `updateInputs(inputs)` poll that fills an `@AutoLog` inputs container once per cycle, plus the output setters (`setMotorVoltages`, `zeroEncoders`).
- `ElevatorIOTalonFX` is the real-hardware implementation — all Phoenix 6 details live there; sensor reads are batched into one `BaseStatusSignal.refreshAll` transaction per cycle.
- `ElevatorSubsystem`'s `periodic()` runs exactly once per 20 ms loop and does three things in order: **poll** the NetworkTables tuning entries (sanitized once, shared by the whole cycle), **poll** the sensors via `io.updateInputs(inputs)` and log them with `Logger.processInputs`, then run the control logic against those snapshots.
- `RobotContainer` is wiring only: the IO implementation, the one fill-in limit from `Constants`, and the button bindings. It holds no tuning data.
- `FakeElevatorIO` (in the tests) implements the same interface with plain fields, so all subsystem logic is tested without hardware.

### Build, Test, and Deploy

Requirements:

- WPILib 2026
- Java 17 from the WPILib 2026 installation
- Phoenix 6 and AdvantageKit vendor dependencies included in `vendordeps`

Build the project:

```sh
./gradlew build
```

Run the automated tests:

```sh
./gradlew test
```

Deploy with the WPILib VS Code command **WPILib: Deploy Robot Code**, or run:

```sh
./gradlew deploy
```

### Main Project Structure

```text
src/main/java/frc/robot/
├── Constants.java
├── Main.java
├── Robot.java
├── RobotContainer.java
└── subsystems/
    ├── ElevatorIO.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

Subsystem tests are located in `src/test/java/frc/robot/subsystems/`. Daily bilingual engineering logs are located in `工程日志/`.

## 中文

### 项目简介

ElevatorDemo 是 11019 X.PLORE 的 WPILib 2026 Java 项目。项目使用 Kraken X60 电机和 TalonFX 内置编码器控制双电机电梯，并包含软件行程限位、AdvantageKit 日志和 AdvantageScope 遥测——与 ArmDemo 验证过的同一套控制模式，只保留电梯部分。

电梯使用固定运动电压。每个动作只需按一次手柄按键，命令会持续运行，直到到达相应的软件限位。

**只需填一个限位**：电梯运行前唯一需要测量的值是 `Constants.Elevator.MAX_EXTENSION_ROTATIONS`。在这个值填入真实数字之前，子系统处于"未标定"状态，所有运动命令都会拒绝运行。

### 安全警告

> **每次 roboRIO 上电前，必须人工将电梯放到物理零位（完全降下）。** 完全断电重启后 TalonFX 内置编码器会自行从零开始读数，如果上电时机构不在物理零位，软件限位将不准确，并可能损坏机构。
>
> 重启机器人程序（重新部署或软重启）**不会**重新归零编码器：TalonFX 位置在代码重启后保持不变，因此比赛中途软重启时，即使电梯不在零位，软件限位依然准确。

人工调整机构时必须保持机器人 Disabled。机器人 Enabled 后，所有人员都应远离电梯的运动范围。

### 测量唯一限位（首次使用标定）

1. 保持 `MAX_EXTENSION_ROTATIONS = 0.0` 部署本代码——电梯是安全的：它不会动。
2. 在机器人 **Disabled** 状态下，用手把电梯从物理零位摇到最高点，在 AdvantageScope 里读取 `Elevator/ExtensionRotations`（两个 TalonFX 位置经方向修正后的平均值）。也可以通电测量：先临时填一个宽松的超估计（如 `999`）作为限位，短暂伸出，及时 Disable 后读取同一个值再覆盖——此时软件限位还不保护你，务必随时准备按 Disable。
3. 把这个数填进 `Constants.Elevator.MAX_EXTENSION_ROTATIONS` 并重新部署。标定到此完成。

填入值的符号就是测得的伸出方向：如果机构绕线方向相反，测出来是负数——照实填入即可，所有电压符号会自动跟随。值为 `0.0`（或任何误输入）时电梯保持拒绝运动。

### 硬件配置

| 电机 | CAN ID | 停止模式 | 正向伸出时的编码器方向 |
| --- | ---: | --- | --- |
| Kraken X60 / TalonFX 1 | 11 | Brake | 原始值减小，软件中取反 |
| Kraken X60 / TalonFX 2 | 12 | Brake | 原始值减小，软件中取反 |

驾驶员手柄连接到 USB 端口 0。虽然代码使用 `CommandPS5Controller`，但当前手柄在 Driver Station 中返回 Xbox 风格的原始按键编号。

### 手柄按键

| 原始按键 | 动作 | 命令行为 |
| ---: | --- | --- |
| 3 | 电梯伸出 | 持续运行，直到任一电机到达唯一上限 |
| 1 | 电梯收回 | 持续运行，直到任一电机回到零位下限 |
| 6 | 电梯中位 | 从任意一侧运行，直到任一电机到达中间工作行程份额（默认半行程） |

所有按键都使用 `onTrue` 绑定，因此按一次即可启动命令，不需要一直按住。每个运动命令还会记录自己的运行时长：完成后在机器人控制台打印一行日志，并作为遥测量（`LastMoveDurationSeconds` / `LastMoveActionName`）发布。

### 电梯控制与限位

- 电机 CAN ID：11 和 12
- 固定运动电压：`2V`
- 伸出输出：电机 1 为 `+2V`（乘以测得的方向符号），电机 2 为其相反数
- 收回输出：符号相反
- 测量：两个 TalonFX 位置分别做方向修正（伸出为正），统一对比唯一限位
- 下限：标定零位（`0` 圈）
- 上限：`MAX_EXTENSION_ROTATIONS`——唯一需要填的值
- 中间工作行程（按键 6）：标定行程的 `MID_EXTENSION_FRACTION`（默认一半）

任一电机先到达上限或下限时，两台电机都会停止。中位命令在启动时锁存一次方向，因此即使一个周期内跳过了到位窗口也不会反向振荡——由远端硬限位结束这次运行。

**标定门控**：`MAX_EXTENSION_ROTATIONS` 为 `0.0`（未标定）时，`extend()`、`retract()`、`moveTowardMidExtension()` 以及全部三个运动命令都会立即以零电压停止。这正是"填一个限位就能用"的安全所在：没填的常量永远驱动不了机构。

### 速度与加速度限制

保留验证过的开环电压运动；规格要求的"可设置并限制最大速度、最大加速度"以可在线调节的安全上限形式叠加在之上：

- **加速度限制**：运动电压每秒最多变化可配置的伏特数（`Elevator Voltage Slew (V/s)`，默认 `60 V/s`——对 2V 而言近似瞬时）。斜坡作用于带符号的电压，因此换向时电压会经过零点渐变，不会瞬间翻转到反极性。
- **速度限制**：当测量到的运动在指令方向上已经达到上限时，运动电压立即归零。每个电机的伸出速度按唯一共享行程归一化，取较快的一侧对比 `Elevator Max Travel (/s)`（默认每秒 1.5 倍全行程）。
- 任何一项设为 `0` 即关闭该限制；误输入的非数值或负值会回退到已验证的默认值，而不会静默关闭限制。

### 动作计时

每个运动命令（伸/收/中位）测量自己的运行时长，完成后在机器人控制台打印 `Elevator action <名称> took <秒> s`，并在 `RealOutputs/Elevator` 下发布 `LastMoveDurationSeconds` 和 `LastMoveActionName`——对应规格里"每个动作均记录完成时间"的要求。

### AdvantageScope 遥测

电梯数据发布在 `RealOutputs/Elevator` 下，包括：

- `Motor1ExtensionRotations` / `Motor2ExtensionRotations`（方向修正后，分电机）
- `ExtensionRotations`（平均值——测量限位时读这个）
- `RawMotor1Rotations` / `RawMotor2Rotations`
- `ExtensionFraction`（标定行程的平均份额）
- `Motor1AppliedVoltage` / `Motor2AppliedVoltage`
- `AtLowerLimit` / `AtUpperLimit` / `Calibrated`
- `MaxExtensionRotations`（填入的限位，便于查看）
- `Motor1TravelFractionPerSecond` / `Motor2TravelFractionPerSecond`
- `LastMoveDurationSeconds` / `LastMoveActionName`

### 数据流（AdvantageKit IO 模式）

同一条数据流：**硬件 → IO → 子系统 → 硬件**。

- `ElevatorIO`（顶层接口）定义硬件契约：一个每周期调用一次的 `updateInputs(inputs)` 轮询，把传感器值填进 `@AutoLog` 输入容器；加上输出方法（`setMotorVoltages`、`zeroEncoders`）。
- `ElevatorIOTalonFX` 是真机实现——所有 Phoenix 6 细节都锁在这个文件里；传感器读取每周期用一次 `BaseStatusSignal.refreshAll` 事务批量完成。
- `ElevatorSubsystem` 的 `periodic()` 在 20 ms 循环里恰好执行一次，按固定顺序做三件事：**轮询** NetworkTables 调参项（每周期消毒一次、全周期共享同一份值），通过 `io.updateInputs(inputs)` **轮询**传感器并用 `Logger.processInputs` 整体记录，然后基于这些快照运行控制逻辑。
- `RobotContainer` 只负责接线：IO 实现、来自 `Constants` 的唯一限位、按键绑定。它不持有任何调参数据。
- 测试替身（`FakeElevatorIO`）用普通字段实现同样的接口，因此全部子系统逻辑都可以在没有硬件的情况下测试。

### 构建、测试与部署

环境要求：

- WPILib 2026
- WPILib 2026 自带的 Java 17
- `vendordeps` 中已经包含 Phoenix 6 和 AdvantageKit 依赖

构建项目：

```sh
./gradlew build
```

运行自动化测试：

```sh
./gradlew test
```

可以使用 VS Code 中的 **WPILib: Deploy Robot Code** 命令部署，也可以运行：

```sh
./gradlew deploy
```

### 主要项目结构

```text
src/main/java/frc/robot/
├── Constants.java
├── Main.java
├── Robot.java
├── RobotContainer.java
└── subsystems/
    ├── ElevatorIO.java
    ├── ElevatorIOTalonFX.java
    └── ElevatorSubsystem.java
```

子系统测试位于 `src/test/java/frc/robot/subsystems/`，每日双语工程日志位于 `工程日志/`。
