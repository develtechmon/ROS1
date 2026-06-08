# MicoAir743v2 — Custom PX4 v1.14.3 HITL Firmware Build Guide

**Target board:** MicoAir H743 V2  
**Firmware branch:** `micoair743-v1.14.3` (Minderring/PX4-Autopilot fork)  
**Host OS:** Ubuntu 20.04 (WSL2 or dual-boot)  
**Purpose:** Enable `pwm_out_sim` module for Hardware-in-the-Loop (HITL) simulation with Gazebo Classic

---

## Background

The MicoAir-provided pre-built PX4 v1.14.3 binary does not include the `pwm_out_sim` module. This module is required for HITL simulation — without it, `HIL_ACTUATOR_CONTROLS` messages from Gazebo have nowhere to go and the simulated motors never spin. The fix is to build the firmware from source with `pwm_out_sim` compiled in.

The correct source is the MicoAir manufacturer fork (`Minderring/PX4-Autopilot`), not mainline PX4, because it contains the board-specific drivers and config for `micoair_h743-v2`.

---

## Prerequisites

- Ubuntu 20.04 (WSL2 or dual-boot)
- `arm-none-eabi-gcc` toolchain installed
- `python3`, `pip3` available
- Internet connection for initial clone and submodules
- QGroundControl installed on Windows host (for flashing and MAVLink console)

---

## Step 1 — Clone the MicoAir Fork

```bash
cd ~
git clone --branch micoair743-v1.14.3 --depth 1 \
  https://github.com/Minderring/PX4-Autopilot.git \
  PX4-Autopilot

cd PX4-Autopilot
git submodule update --init --recursive
```

> `--depth 1` keeps the initial download small. Submodule sync takes 5–10 minutes depending on connection speed.

---

## Step 2 — Fix the Shallow Clone Tag Issue

The `--depth 1` flag means no tags are fetched by default. CMake requires a valid tag to parse the firmware version string. Without this step, the build fails at `CMakeLists.txt:129`.

```bash
cd ~/PX4-Autopilot
git fetch --unshallow
git fetch --tags
```

Verify:

```bash
git describe --tags
```

Expected output: `v1.14.3`

---

## Step 3 — Install kconfiglib (required for boardconfig GUI)

```bash
pip3 install kconfiglib --break-system-packages
```

---

## Step 4 — Verify Board Target Exists

```bash
ls boards/micoair/h743-v2/
```

Expected output includes: `default.px4board`

If this folder is missing, the wrong repo or branch was cloned. Stop and re-check Step 1.

---

## Step 5 — Enable pwm_out_sim via boardconfig

Launch the interactive board configuration tool:

```bash
cd ~/PX4-Autopilot
make micoair_h743-v2_default boardconfig
```

Navigate to:

```
Modules →
  Simulation →
    [ ] pwm_out_sim    ← press SPACE to enable → [*] pwm_out_sim
```

Save and exit: press **S** to save, **ESC** to go back, **Q** to quit.

Verify the config was written:

```bash
grep "PWM_OUT_SIM" boards/micoair/h743-v2/default.px4board
```

Expected output:

```
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
```

> If the line is missing, the boardconfig GUI did not save correctly. Add it manually:
> ```bash
> echo "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" >> boards/micoair/h743-v2/default.px4board
> ```

---

## Step 6 — Build the Firmware

```bash
make clean
make micoair_h743-v2_default -j$(nproc) 2>&1 | tail -20
```

A successful build ends with a memory usage table:

```
Memory region         Used Size  Region Size  %age Used
           FLASH:     xxxxxxx B      1792 KB     xx.xx%
```

### If flash overflows

The H743 has 1792 KB flash. If the build fails with `region 'flash' overflowed`, remove the fixed-wing position control module — it is unused on a multicopter HITL build:

```bash
sed -i 's/CONFIG_MODULES_FW_POS_CONTROL=y/CONFIG_MODULES_FW_POS_CONTROL=n/' \
  boards/micoair/h743-v2/default.px4board

make clean
make micoair_h743-v2_default -j$(nproc) 2>&1 | tail -20
```

---

## Step 7 — Locate the Built Firmware

The compiled firmware `.px4` file is at:

```
~/PX4-Autopilot/build/micoair_h743-v2_default/micoair_h743-v2_default.px4
```

If building inside WSL2, access the file from Windows Explorer at:

```
\\wsl$\Ubuntu-20.04\home\<username>\PX4-Autopilot\build\micoair_h743-v2_default\micoair_h743-v2_default.px4
```

Replace `<username>` with your actual Linux username.

---

## Step 8 — Flash the Firmware via QGroundControl

1. Open QGroundControl on the Windows host.
2. Go to **Vehicle Setup → Firmware**.
3. Plug in the MicoAir743v2 via USB.
4. Select **Advanced settings → Custom firmware file**.
5. Browse to `micoair_h743-v2_default.px4` from the path above.
6. Click **OK** and wait for the flash to complete.

---

## Step 9 — Verify pwm_out_sim is Present

In QGC, open **Analyze Tools → MAVLink Console** and run:

```
nsh> pwm_out_sim status
```

If the module is present, you will see a status output (or a message that it is not running). Either result confirms the module exists in the firmware.

If you still see `command not found`, the boardconfig change did not make it into the build — go back and confirm Step 5 before rebuilding.

---

## Step 10 — Set HITL Parameters

Re-enter all HITL parameters after the fresh flash (flashing wipes all saved parameters):

```
nsh> param set SYS_HITL 1
nsh> param set SYS_AUTOSTART 1001
nsh> param set CBRK_FLIGHTTERM 121212
nsh> param set EKF2_GPS_CTRL 7
nsh> param set EKF2_HGT_REF 0
nsh> param save
nsh> reboot
```

After reboot, the board is ready for HITL simulation with Gazebo Classic.

---

## Parameter Reference

| Parameter | Value | Purpose |
|---|---|---|
| `SYS_HITL` | `1` | Enable HITL mode |
| `SYS_AUTOSTART` | `1001` | HIL Quadcopter X airframe |
| `CBRK_FLIGHTTERM` | `121212` | Disable flight termination circuit breaker |
| `EKF2_GPS_CTRL` | `7` | GPS fusion control for EKF2 |
| `EKF2_HGT_REF` | `0` | Barometric height reference |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `pwm_out_sim: command not found` | Module not compiled in | Confirm `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y` in `.px4board`, rebuild |
| `fatal: No names found` on `git describe` | Shallow clone missing tags | Run `git fetch --unshallow && git fetch --tags` |
| `CMake Error at CMakeLists.txt:129` | No valid tag for version parsing | Same as above |
| `region 'flash' overflowed` | Firmware too large for 1792 KB | Disable `CONFIG_MODULES_FW_POS_CONTROL` and rebuild |
| `boardconfig` change not saved | Exited GUI without saving | Press **S** before **Q**, or add the line manually with `echo` |
| Gazebo motors not spinning in HITL | `pwm_out_sim` not running | Verify module present, verify `SYS_HITL=1` and `SYS_AUTOSTART=1001` after reboot |

---

## Notes

- This guide applies specifically to the **MicoAir743v2** board. The `Minderring/PX4-Autopilot` fork is the manufacturer's source — do not use mainline `PX4/PX4-Autopilot` as it may not have the correct board drivers for this target at v1.14.3.
- The `1001_rc_quad_x.hil` airframe file only excludes `px4_fmu-v2` — the MicoAir board has no explicit exclude entry and is enabled by default.
- Parameters are wiped on every firmware flash. Save a parameter file in QGC before flashing if you have additional tuning values to preserve.
