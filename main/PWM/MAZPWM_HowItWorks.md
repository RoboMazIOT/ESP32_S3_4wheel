# How MAZPWM Enables PWM to Drive a DC Motor
## ESP32-S3 Bare-Metal MCPWM — Step-by-Step

---

## Overview

A DC motor driven by a DRV8833 H-bridge needs two logic signals: **IN1** and **IN2**.
PWM duty on IN1 (with IN2 = 0) spins the motor forward at a speed proportional to duty %.
The ESP32-S3 **MCPWM peripheral** generates those PWM signals entirely in hardware — the CPU
only writes a compare value; the peripheral handles the waveform with zero CPU overhead.

The path from `init()` to a spinning motor has **four stages**:

```
Stage 1: Clock Gate      → peripheral powered on, registers accessible
Stage 2: Timer Config    → counting engine set up (frequency, mode)
Stage 3: Operator Config → comparator + generator waveform rules
Stage 4: GPIO Routing    → signal wired to physical pin
```

---

## Stage 1 — Enable the MCPWM Clock Gate

**File:** [`MAZPWM.cpp`](MAZPWM.cpp) — `_enableClock(unitIdx)`

The ESP32-S3 starts with most peripheral clocks **off** to save power. Writing to the
MCPWM registers before enabling the clock produces no effect — reads return garbage,
writes are silently discarded.

The clock gate lives in the **DPORT system register** `SYSTEM_PERIP_CLK_EN0_REG`
(address `DR_REG_SYSTEM_BASE + 0x18`):

| Bit | Field            | Peripheral |
|-----|------------------|------------|
| 17  | `pwm0_clk_en`    | MCPWM0     |
| 20  | `pwm1_clk_en`    | MCPWM1     |

> **The critical lesson learned:** These bits are in register **0** (`_CLK_EN0`),
> not register 1 (`_CLK_EN1`). Writing to the wrong register silently did nothing —
> the clock was never on, the timer never ran, no PWM ever appeared.

### Init sequence (matches `esp_hal_mcpwm` LL):

```
1. SET   SYSTEM_PERIP_CLK_EN0_REG  bit 17  → clock ON
2. SET   SYSTEM_PERIP_RST_EN0_REG  bit 17  → assert reset  (flushes stale state)
3. CLEAR SYSTEM_PERIP_RST_EN0_REG  bit 17  → release reset (peripheral starts clean)
```

After step 3 the MCPWM register file is accessible and all registers are at their
reset defaults (timers frozen, generators idle, all outputs LOW).

We also set `dev->clk.clk_en = 1` in `_initUnit()` — this is the peripheral's own
internal APB register-file clock (separate from the DPORT gate above). Both must be on.

---

## Stage 2 — Configure the Timer

**File:** [`MAZPWM.cpp`](MAZPWM.cpp) — `_initUnit(dev)`, timer loop

Each MCPWM unit (MCPWM0, MCPWM1) has **3 independent timers**. Each timer drives one
operator (= one motor channel). Motor 0 uses timer 0, motor 1 uses timer 1, etc.

### Frequency calculation

```
Source clock           = 160 MHz  (APB on ESP32-S3 at 240 MHz CPU)
Global prescaler       = 7        → timer_clock = 160 MHz / (7+1) = 20 MHz
Timer prescaler        = 0        → no additional division
PWM period ticks       = 1000     → PWM freq = 20 MHz / 1000 = 20 kHz
```

20 kHz is above human hearing — the motor makes no audible whine.

### Register writes

```
timer_cfg0.timer_prescale        = 0      // no per-timer division
timer_cfg0.timer_period          = 999    // counter wraps at 999 (1000 steps)
timer_cfg0.timer_period_upmethod = 0      // period change takes effect immediately
timer_cfg1.timer_mod             = 1      // up-count mode (0→999→0→...)
timer_cfg1.timer_start           = 2      // start now, run forever (free-run)
```

### What the timer does after this

```
tick:   0    1    2  ...  599  600  ...  999  0    1   (wraps)
count:  0    1    2  ...  599  600  ...  999  0    1
              ↑                  ↑              ↑
           TEZ event         compare A      period end
         (timer=zero)        (at 60%)       → wraps to 0 → TEZ again
```

`timer_start = 2` is a **self-clearing** write command — hardware executes it and the
register returns to 0. The timer continues running; you never need to write it again.

---

## Stage 3 — Configure the Operator (Comparator + Generator)

**File:** [`MAZPWM.cpp`](MAZPWM.cpp) — `_initUnit(dev)`, operator loop

Each operator contains:
- A **comparator** (two compare registers: A for CW pin, B for CCW pin)
- A **generator** (two generators: gen-A drives CW pin, gen-B drives CCW pin)

### 3a — Bind operator to its timer

```
operator_timersel.operator0_timersel = 0   // op0 → timer0
operator_timersel.operator1_timersel = 1   // op1 → timer1
operator_timersel.operator2_timersel = 2   // op2 → timer2
```

This tells each operator which timer's TEZ/TEP events to respond to.

### 3b — Comparator shadow update method

```
gen_stmp_cfg.gen_a_upmethod = 0   // compare A value takes effect immediately
gen_stmp_cfg.gen_b_upmethod = 0   // compare B value takes effect immediately
```

`upmethod = 0` (immediate) means: when `setMotorDuty()` writes a new value to
`timestamp[0].gen`, it takes effect on the **very next timer tick** — no waiting for a
TEZ event. This gives the lowest possible latency when changing motor speed.

### 3c — Generator action table

This is the heart of the PWM waveform. We tell each generator exactly what to do on
two events:

**Generator A (CW pin):**

| Event                       | Action    | Register field |
|-----------------------------|-----------|----------------|
| TEZ — timer reaches zero    | Force HIGH | `gen_utez = 2` |
| TEA — timer reaches compare A | Force LOW | `gen_utea = 1` |
| All other events            | No change  | (val = 0 clears them) |

**Generator B (CCW pin):** Same pattern using compare B (`gen_uteb`).

### The resulting waveform (compare A = 600, duty = 60 %)

```
         period = 1000 ticks (50 µs at 20 kHz)
         ┌─────────────────────────────────────────────────────┐
Output:  │████████████████████████████████████│                │
         │         HIGH (600 ticks)           │  LOW (400 tks) │
         0                                  600              999 → wraps
         ↑                                    ↑
       TEZ fires → HIGH                    TEA fires → LOW
```

duty % = (compare_value / 1000) × 100  →  600/1000 = **60 %**

### 3d — Initial compare values = 0 (coast on startup)

```
operators[i].timestamp[0].gen = 0;   // compare A = 0 → CW pin always LOW
operators[i].timestamp[1].gen = 0;   // compare B = 0 → CCW pin always LOW
```

Both pins LOW = DRV8833 coast (motor freewheels). Safe power-on default.

---

## Stage 4 — Route Signals to Physical GPIO Pins

**File:** [`MAZPWM.cpp`](MAZPWM.cpp) — `_routeGpio(gpio, signalIdx)`

The ESP32-S3 GPIO matrix lets any internal peripheral signal reach any GPIO pad.
Three writes are required per pin:

### Step 1 — IO MUX: switch pad to GPIO matrix mode

```cpp
PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);  // func = 1
```

Each GPIO pad has a direct IO MUX connection (high speed, bypasses matrix) and a
matrix path. `PIN_FUNC_GPIO = 1` selects the matrix path for this pad.

### Step 2 — GPIO matrix: select which signal drives this pad

```cpp
GPIO.func_out_sel_cfg[gpio].func_sel = signalIdx;
```

`signalIdx` comes from `gpio_sig_map.h` for ESP32-S3:

| Motor | Peripheral   | CW signal index | CCW signal index |
|-------|--------------|-----------------|-----------------|
| 0 FL  | MCPWM0 op 0  | `PWM0_OUT0A_IDX` = 160 | `PWM0_OUT0B_IDX` = 161 |
| 1 FR  | MCPWM0 op 1  | `PWM0_OUT1A_IDX` = 162 | `PWM0_OUT1B_IDX` = 163 |
| 2 RL  | MCPWM0 op 2  | `PWM0_OUT2A_IDX` = 164 | `PWM0_OUT2B_IDX` = 165 |
| 3 RR  | MCPWM1 op 0  | `PWM1_OUT0A_IDX` = 166 | `PWM1_OUT0B_IDX` = 167 |

### Step 3 — Enable output driver

```cpp
// GPIOs 0–31:
GPIO.enable_w1ts = (1U << gpio);          // plain uint32_t — direct write

// GPIOs 32+:
GPIO.enable1_w1ts.val = (1U << (gpio - 32)); // union field — needs .val
```

`enable_w1ts` is a **write-one-to-set** register. Writing a 1 to a bit enables that
GPIO's output buffer. The pad will now be driven by the MCPWM signal.

---

## Putting It All Together — Motor Forward at 60 %

```
app_main()
  └─ robot.begin(DriveMode::PWM)
       ├─ gpio_reset_pin() × 8          → all motor pins to clean input state
       ├─ _pwm.init()
       │    ├─ _enableClock(0)          → MCPWM0 clock ON + reset pulse
       │    ├─ _initUnit(&MCPWM0)       → 3 timers + 3 operators configured
       │    ├─ _enableClock(1)          → MCPWM1 clock ON + reset pulse
       │    └─ _initUnit(&MCPWM1)       → timer 0 + operator 0 configured
       └─ attachMotorPin(0..3, cw, ccw) → GPIO matrix routes 8 signals to 8 pins

  └─ robot.moveForward(60.0f)
       └─ _applyMotorVector(+60, +60, +60, +60)
            └─ _driveMotor(FL, FORWARD, 60.0f)
                 └─ _pwm.setMotorDuty(0, 60.0f, 0.0f)
                      ├─ _dutyToTicks(60.0f) = 600
                      ├─ operators[0].timestamp[0].gen = 600  ← CW  pin: 60% duty
                      └─ operators[0].timestamp[1].gen = 0    ← CCW pin: 0%  (LOW)
```

Hardware then produces on GPIO 39 (FL CW pin):

```
20 kHz, 60% duty:
▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔╱‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾╲
___________ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _|  DRV8833 averages |→ motor spins
▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔  this as 60% Vcc  |
```

GPIO 40 (FL CCW pin) stays LOW the whole time → DRV8833 spins motor forward.

---

## DRV8833 Truth Table (what MAZPWM implements)

| Command  | CW pin (IN1)  | CCW pin (IN2) | Motor state          |
|----------|---------------|---------------|----------------------|
| FORWARD  | PWM duty %    | 0 %           | Forward, variable speed |
| BACKWARD | 0 %           | PWM duty %    | Reverse, variable speed |
| BRAKE    | 100 %         | 100 %         | Fast electromagnetic stop |
| COAST    | 0 %           | 0 %           | Freewheel (power off) |

---

## Key Numbers Quick Reference

| Parameter         | Value    | How it is set                                      |
|-------------------|----------|----------------------------------------------------|
| APB source clock  | 160 MHz  | ESP32-S3 hardware default at 240 MHz CPU           |
| Global prescaler  | ÷8 (val=7) | `clk_cfg.clk_prescale = 7`                       |
| Timer clock       | 20 MHz   | 160 / 8                                            |
| Timer prescaler   | ÷1 (val=0) | `timer_cfg0.timer_prescale = 0`                  |
| Period ticks      | 1000     | `timer_cfg0.timer_period = 999` (count 0…999)      |
| PWM frequency     | 20 kHz   | 20 MHz / 1000                                      |
| Duty resolution   | 0.1 %    | 1 tick out of 1000                                 |
| Timer count mode  | Up       | `timer_cfg1.timer_mod = 1`                         |
| Timer run mode    | Free-run | `timer_cfg1.timer_start = 2` (self-clearing cmd)   |
