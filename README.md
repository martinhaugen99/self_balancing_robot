# Self-Balancing Robot

A two-wheeled self-balancing robot built for an embedded systems project. The
repository covers the whole path from theory to hardware:

1. **Modelling** — the robot is treated as an inverted pendulum on a cart
   (cart-pole). The equations of motion are derived symbolically in MATLAB,
   linearised around the upright equilibrium, and checked for stability,
   controllability and observability.
2. **Simulation** — the nonlinear plant is rebuilt in Simulink and closed with a
   PID controller; sensor noise and a discrete Kalman filter are simulated on
   top of the logged states.
3. **Firmware** — a bare-metal-ish STM32F446RE application reads an MPU-6050
   IMU over a hand-rolled I²C driver, fuses accelerometer and gyroscope data
   with a 2-state Kalman filter, and drives two DC motors through a dual
   H-bridge at a 200 Hz control rate.

---

## Repository layout

| Path | What it is |
| --- | --- |
| `system_setup.m` | First derivation. Builds the equations of motion from force/moment balances (`eliminate`, `odeToVectorField`), simulates the open-loop response and linearises symbolically via Jacobians. |
| `cart_pole.m` | Self-contained derivation + analysis. Solves for `ddx`/`ddtheta`, linearises, compares nonlinear vs linear open-loop response, and runs stability / controllability / observability checks. Ends with an ODE-based PID simulation. |
| `cart_pole_v2.m` | The version that matches the real hardware. Uses measured masses and geometry, builds the state-space model, runs `cart_pole_nonlinear.slx`, then adds synthetic sensor noise and runs a 4-state discrete Kalman filter over the result. |
| `controlled_dynamics.m` | ODE right-hand side used by `cart_pole.m`: computes the PID input from the current angle and returns the state derivative. Keeps controller memory in `persistent` variables. |
| `cart_pole_nonlinear.slx` | Simulink model (R2024b). A `cart-pole` subsystem holds the nonlinear plant built from integrators/trig/product blocks; a continuous-time PID block closes the loop on `theta`. Logs `X` and `u` to the workspace. |
| `plot_angles.py` | Host-side serial reader for live telemetry from the board. |
| `stm/` | STM32CubeIDE project for the STM32F446RE (Nucleo-64 pinout). |

Application code lives in `stm/Core/`:

| File | Role |
| --- | --- |
| `Src/main.c` | Clock setup, TIM3 PWM, TIM6 200 Hz tick, motor direction GPIO, and the control loop. |
| `Src/mpu_6050.c`, `Inc/mpu_6050.h` | Register-level I²C1 driver plus MPU-6050 init, accel/gyro reads and gyro bias calibration. |
| `Src/kalman_filter.c`, `Inc/kalman_filter.h` | 2-state (angle, gyro bias) Kalman filter. |
| `Inc/pid.c`, `Inc/pid.h` | Fixed-timestep PID with integral clamping and output saturation. |
| `Src/gpio.c`, `Src/usart.c` | CubeMX-generated peripheral init (USART2 at 115200 8N1 on PA2/PA3). |

---

## The model

Both MATLAB derivations start from the same two equations — horizontal force
balance on cart + pendulum, and the moment balance about the pendulum's centre
of mass:

```
u - (M + m)·ẍ - m·l·(θ̇²·sin θ - θ̈·cos θ) = 0
(J + m·l²)·θ̈ - m·l·(ẍ·cos θ + g·sin θ)   = 0
```

These are solved for `ẍ` and `θ̈`, assembled into `f(x, u)`, and linearised by
Jacobian evaluation at `x = 0, u = 0`.

Parameters used in `cart_pole_v2.m` (measured from the built robot):

| Symbol | Value | Meaning |
| --- | --- | --- |
| `M` | 0.27 kg | cart (wheel/axle) mass |
| `m` | 0.69 kg | total body mass (chassis 0.37 + power bank 0.15 + driver 0.03 + STM board 0.04 + breadboard 0.10) |
| `l` | 0.0675 m | half the rod length (0.135 m) |
| `J` | 0.006767 kg·m² | composite body inertia |
| `g` | 9.81 m/s² | gravity |

The analysis in `cart_pole.m` gives three results worth keeping in mind:

- **Unstable** — the linearised system has a pole in the right half-plane
  (≈ +5.4 rad/s with the `cart_pole.m` parameters, ≈ +7.7 rad/s with the
  hardware parameters in `cart_pole_v2.m`), plus a double pole at the origin
  from the cart-position integrator chain.
- **Controllable** — `rank(ctrb) = 4`, so every state can in principle be
  driven by the single input. A full state-feedback design (LQR, pole
  placement) is available.
- **Not observable from the angle alone** — `rank(obsv) = 2`. Cart position and
  velocity do not appear in any of the state derivatives, so no angle
  measurement can reconstruct them. This is not a modelling slip; it is the
  reason an angle-only robot with no encoders drifts along the floor no matter
  how well the angle loop is tuned.

### Running the simulations

Requires MATLAB R2024b with **Symbolic Math Toolbox**, **Control System
Toolbox** and **Simulink**.

```matlab
>> cart_pole        % derivation, open-loop comparison, system analysis
>> cart_pole_v2     % hardware parameters, Simulink run, Kalman filter
```

`cart_pole_v2.m` must be run before (or instead of) opening the `.slx` directly
— the model reads `X_0`, `theta_ref` and `T_sim` from the base workspace.

---

## Firmware

### Hardware

- **MCU**: STM32F446RE, Nucleo-64 (user button on PC13, LD2 on PA5)
- **IMU**: MPU-6050 on I²C1 (address `0x68`), ±2 g / ±250 °/s, 1 kHz internal
  sample rate
- **Motor driver**: dual H-bridge (L298N-style — two PWM enables, four
  direction inputs)
- **Actuators**: 2× DC gearmotors
- Powered from a USB power bank

### Pin map

| Pin | Function |
| --- | --- |
| PB8 / PB9 | I²C1 SCL / SDA (AF4, open-drain, pull-up) — MPU-6050 |
| PA6 | TIM3_CH1 PWM — motor 1 enable |
| PA7 | TIM3_CH2 PWM — motor 2 enable |
| PC0 / PC1 | Motor 1 direction (IN1 / IN2) |
| PC2 / PC3 | Motor 2 direction (IN3 / IN4) |
| PA2 / PA3 | USART2 TX / RX — telemetry over the ST-Link VCP |

### Clocks and timing

`SystemClock_Config()` runs the PLL from the 16 MHz HSI
(`M=16, N=336, P=4`) for a **84 MHz** SYSCLK; APB1 is HCLK/2, so the APB1 timers
still see 84 MHz.

| Timer | Prescaler / ARR | Result |
| --- | --- | --- |
| TIM6 | 84 / 5000 | 200 Hz control-loop interrupt (`do_control` flag) |
| TIM3 | 84 / 200 | 5 kHz PWM on CH1 and CH2 |

### Control loop

TIM6's ISR does nothing but set a flag; the work happens in `main()`'s
superloop so the ISR stays short. Each 5 ms tick:

1. Read the accelerometer, compute pitch as
   `atan2(Ax, √(Ay² + Az²))` in degrees.
2. Read the gyroscope, subtract the bias measured at startup by
   `MPU6050_Calc_Offset()` (200 samples averaged), and integrate for a
   drift-prone gyro-only angle.
3. Compute a complementary-filter estimate (`α = 0.98`) — kept for comparison,
   not used for control.
4. Run the Kalman filter (`Q_angle = 0.01`, `Q_bias = 0.003`,
   `R_measure = 0.01`) to fuse the accelerometer angle with the gyro rate.
   **This is the estimate the controller uses.**
5. Feed the estimate to the PID controller (setpoint 0°, upright).
6. Set motor direction from the sign of the output, apply `|output|` as the
   PWM compare value with a 150-count deadband, and write both compare
   registers.

### Gains

| Where | kp | ki | kd | Notes |
| --- | --- | --- | --- | --- |
| Firmware (`main.c`) | 40 | 70 | 0.5 | hand-tuned on the robot; error in **degrees**, output in PWM counts |
| Simulink PID block | 414.8 | 2449 | 17.56 (N = 100) | tuned against the model; error in **radians**, output in newtons |

The two sets are not comparable — different units, different plant, different
actuator model. The firmware gains were not derived from the simulation.

### Building and flashing

Open `stm/` in **STM32CubeIDE** and build the `Debug` configuration, then flash
over ST-Link. The project links against `STM32F446RETX_FLASH.ld` with
hard-float (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`) and newlib-nano.

> The checked-in `stm/Debug/makefile` contains absolute paths from the machine
> it was generated on. Let CubeIDE regenerate it rather than invoking `make`
> directly.

---

## Telemetry

`plot_angles.py` opens the board's virtual COM port and streams lines from the
firmware:

```bash
pip install pyserial matplotlib
python plot_angles.py
```

Two things to fix before it does anything useful:

- The port is hard-coded to `/dev/tty.usbmodem11303` (macOS). Change it to your
  own port.
- The firmware's `printf`/`HAL_UART_Transmit` calls in `main.c` are currently
  commented out, so nothing is transmitted. Uncomment them — and note that
  `__io_putchar` is only defined `weak` in `syscalls.c`, so `printf` needs a
  real retarget to reach USART2.

The parsing and plotting half of the script is also commented out; it currently
just echoes raw lines.

---

## Known issues and limitations

These are real, and worth knowing before trusting the robot or the numbers:

- **PWM range mismatch.** `TIM3->ARR` is 199, but the PID output saturates at
  1000 and the initial compare values are set to 499 ("50% duty"). Anything
  above 199 is full duty, and the 150-count deadband cuts everything below it —
  so the usable range is roughly 150–199 out of 199. In practice the drive is
  close to bang-bang rather than proportional. Either raise `ARR` to ~1000 or
  scale the PID output down to the ARR range.
- **I²C reads dominate the control period.** `MPU6050_Read_Accel()` and
  `MPU6050_Read_Gyro()` each issue six single-byte register reads instead of one
  6-byte burst. At the configured bus speed that is roughly 0.4 ms per read,
  ~5 ms for all twelve — about the entire 5 ms budget (back-of-the-envelope; worth
  measuring with a scope or a cycle counter). It also means the six bytes of an
  axis triplet are not sampled atomically. A single burst read would fix both.
- **I²C clock configured for the wrong PCLK1.** `CR2` is set to 45 MHz and
  `CCR`/`TRISE` to 225/46, but PCLK1 is 42 MHz here — SCL ends up near 93 kHz
  instead of 100 kHz.
- **`TIM6_basic_setup()` writes `TIM3->EGR`** where it means `TIM6->EGR`, so
  TIM6's prescaler shadow register is not preloaded before the timer starts.
- **No fall detection or wheel feedback.** There are no encoders, so cart
  position/velocity are unobservable on the hardware — the firmware controls
  angle only, and the robot will drift. The state-feedback story in the MATLAB
  model has no hardware counterpart.
- **`cart_pole.m`'s PID section is inert** — `kp`, `ki`, `kd` are all set to 0
  (the Ziegler–Nichols values are commented out), so that simulation runs
  open-loop. The `U` vector it plots is also hard-coded to zeros rather than
  recording the actual control effort.
- **The Kalman filter in `cart_pole_v2.m` assumes a uniform timestep it does
  not have.** The model runs on a variable-step solver, so the logged `t` is
  non-uniform, but the filter discretises with a fixed `dt = 0.0005` and steps
  once per logged sample. Either switch the model to a fixed-step solver at
  that rate, or rebuild `A_d`/`B_d` per step from the actual `diff(t)`.
  Relatedly, with `theta` as the only output the position states are
  unobservable, so the filter never corrects them — they propagate open-loop.
- **The stability print-out in `cart_pole.m` is misleading.** The loop prints
  "System is stable" once per eigenvalue until it reaches one with a positive
  real part, so a single run can print both messages. It also counts the poles
  at the origin as stable rather than marginal.

## Possible next steps

- Add wheel encoders and close a position loop around the angle loop.
- Replace the hand-tuned PID with LQR or pole placement using the `A`/`B` from
  `cart_pole_v2.m` — the model is already there and the system is controllable.
- Burst-read the IMU and move the control loop into the timer ISR.
- Re-enable UART telemetry and finish `plot_angles.py` for live tuning.

## Repository hygiene

`stm/Debug/` (~25 MB of `.o`, `.elf`, `.map` and generated makefiles) and
`slprj/` (Simulink cache) are committed. They are build outputs and would be
better ignored:

```gitignore
stm/Debug/
slprj/
*.slxc
```

## License

The project code has no license file. Vendored components keep their own terms:
STMicroelectronics' HAL drivers and CMSIS under `stm/Drivers/` are covered by
`stm/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt`.
