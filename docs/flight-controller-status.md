# Ulysses flight controller: status report

**As of 22 Nov 2026.** Branch `dev/furqazaheervi6/controller-handle` (head `4a174fa`), PR #17 on the private repo `furqazaheervi6/ulysses`. PR #17 was merged into `main` as `23f7b1b` with a merge commit, so every commit and author is preserved.

## Summary

The controller's structure is in place: a caller-owned handle, safe gain updates from another task, time-step guards, PID with anti-windup, and the quaternion helpers. CI is green on the latest head, `4a174fa`: the ARM Debug and Release builds, lint, and the all-builds check. It now builds `flight_controller.c` and `gimbal.c`. **The control law itself is not written**, because it waits on frame definitions from the controls team. Of the 16 issues, 10 are closed. Of the 6 that remain, most wait on another team or a teammate's branch. The 22 Nov decisions in the update section below change #8, #12, #13 and #15.

## At a glance: what is needed, from whom, and why

Before the 22 Nov conversation, all seven open issues were waiting on someone else. The update section further down records what changed.

| # | Issue | What is needed | From | Why |
|---|---|---|---|---|
| **7** | Frames and gimbal signs | World and body axes (the ESKF branch's forward/right/down conflicts with a thrust axis of +z), which frame omega and the inertia are in, where frames are converted, units, and the physical servo direction | **Controls team** | The control law mixes `q_err` (a world-frame rotation), omega, the inertia and the thrust direction. Without the frames, a sign error could tilt the gimbal the wrong way. |
| **8** | Renormalize after the gyro update | When `eskf_predict` gets written, and whether to normalize after the measurement update too | **ESKF owner** | The normalize call goes inside `eskf_predict`, which is an empty stub. |
| **11** | Thrust to motor layer | Which ESC flies, its protocol and speed, and whether changing the linker script and DMA setup is OK. Also the fault output and minimum spin throttle, the table input units, the characterization data and grid, and which motor is "top" | **Firmware lead**, **controls team**, **mechanical and simulation team** | There is no data to fill the tables with, and the driver cannot be written for an unknown ESC. The CM7 also has no DMA-reachable RAM. |
| **12** | `pos_ref` / `vel_ref` | How a position or velocity error becomes a thrust demand (the references themselves are hard-coded for the hop test) | **Controls team** | #11's tables need a thrust input. The fields stay, but no doc defines the vertical or position loop that produces the thrust. |
| **13** | `fc_config_t` from CAD | Mass, inertia tensor, centre of mass and gimbal pivot (with reference point, axes and CAD revision), the body frame, and the PID gains. The gimbal limit is about 18 degrees, an untested estimate | **Mechanical team**, plus **controls team** for the frame and gains | The config has no values or instance yet. A zeroed config means zero mass and zero gains, with no error. |
| **15** | `eskf.h` and state type | Closed, and CI is green. Later, an adapter from the estimator's output to `fc_state_t` | **ESKF owner** (later) | The controller now takes its own state struct, so it no longer depends on `eskf.h`. Omega is stubbed until the estimator provides it. |
| **16** | Joseph form and EKF typos | A real update step, whether the baro bias is in the state, and whether `h` depends on `u` | **ESKF owner**, **Kenneth Lew** (docs author) | `eskf_update_position` is an empty stub, so there is nothing to convert yet. Only the author can say which `u_k` is wrong. |

**What unblocks the most:** the controls team's answers. They gate #7, #11 and #12, and part of #13. The detailed question lists are further down.

## Update: decisions from the team conversation (22 Nov)

| Issue | What was decided or learned | Effect |
|---|---|---|
| **#15** ESKF state type | Treat the state estimator as a black box. The controller takes its own input struct, and omega is stubbed until the estimator provides it. | Committed and pushed as `a9cd632`: `fc_state_t` (orientation, omega, position, velocity), `step` takes it, the `eskf.h` include is gone, and `flight_controller.c` is added to the CMake build. Host tests pass on clang and gcc, the ARM builds pass in CI, and #15 is closed. |
| **#8** normalize call | The ESKF owner is still working on the filter. | Left as a flag. Resume when `eskf_predict` is written. |
| **#12** `pos_ref` / `vel_ref` | They are the desired values, hard-coded for the hop test (straight up) and set from the console later. The measured values come from the estimator. | The fields stay. Still open: how a position or velocity error becomes a thrust demand, which no doc defines. |
| **#13** gimbal limits | About plus or minus 18 degrees on the real gimbal, a mechanical estimate that is not fully tested and might reach 20. | Use 18 degrees, which is 0.3142 rad. Still missing: mass, inertia, centre of mass, pivot, the body frame and the PID gains. |
| **#7**, **#11** | Which servo is X and which is Y, and which motor is the top one, are not specified. The ESC protocol and speed are still open. | Ask firmware first. |

**Questions in this report that this changes**
- Controls question 4 (where the thrust demand comes from): the references are answered, the thrust law is not.
- ESKF owner question 4 (how omega reaches the controller): answered by the black-box input struct.
- Mechanical question 2 (gimbal limits): an estimate of 18 degrees, to be tested.

**New item, not in any issue: the servo bus**
The old servo code used a blocking 3 ms sleep between sending and receiving, because the bus is half-duplex. The pin sheet has a single "Dynamixel TTL" line, so both gimbal servos probably share one bus. Dynamixel Protocol 2.0 has Sync Write, to command both servos in one packet, and Sync Read, to get both replies back to back, which removes the need for a blocking delay. This is from general knowledge of the protocol, not the team docs, so check it against the exact servo model. It could become its own issue for the servo driver.

## Branch and CI events since 8 Nov

- **Black-box state struct pushed.** `a9cd632` adds `fc_state_t` and removes the controller's dependence on `eskf.h` (#15).
- **A merged PR was already on the branch.** PR #18, "sim: add Simantic simulation test image and CI job", was merged into the branch on 12 Nov. My first push was rejected for that reason, so I rebased my commit on top of it instead of forcing anything.
- **Merge conflict with main, resolved.** The sim job and main's lint job were both added at the end of `.github/workflows/ulysses-firmware.yaml`. GitHub does not run CI on a PR with a merge conflict, which is why CI had not started. I merged main into the branch (`cadd410`) and kept both jobs.
- **CI on `cadd410`.** Build (Debug) and Build (Release) passed, so the controller compiles for the ARM target. Lint failed, on two lines in `ulysses/sim/controls_test.c`, not on any controls file. The simulation job reported success but was skipped, because the `SIMANTIC_EMAIL` secret was never set.
- **Simantic work left out for now.** On your request I reverted the sim PR (`5fd9cc2`): `ulysses/sim/` and the `simulate` job are gone, and the workflow is back to main's version. The work is still in the history at `edc75b0`.
- **CI on `5fd9cc2`:** all four checks passed, so removing the sim test file fixed the lint failure.
- **Gimbal code and flags pushed.** `4a174fa` adds `gimbal.h` and `gimbal.c`, names the gimbal limits in radians, corrects the frame notes in `flight_controller.h`, and flags the known limitations. CI is green on it.
- **#15 closed** with a comment recording the commit and the CI result.

## Done (issues #1 to #6, #9, #10, #14 and #15 closed)

| Area | What exists | Commits |
|---|---|---|
| Handle and state (#1, #3) | `flight_controller_t` owned by the caller, `init` copies the config and clears all state | `47386ea`, `b239a04`, `db936f1` |
| Live gain tuning (#2) | `set_gains` writes a pending buffer; `step` applies it at the start of its next call. Lock-free, so the control loop never blocks, and the integrators are kept | `1b5711e` |
| Time-step safety (#4) | A zero, negative or NaN `dt_s` sets a fault flag and asserts. A late tick is capped at 3 s and flagged | `44e71b9`, `3d68aff`, `5100a0d` |
| Ownership and faults (#5) | One control task owns `init` and `step`; one other task may call `set_gains`. `faults` is atomic | `8072de3` |
| Shortest path (#6) | `quaternion_shortest_path` flips `q` when `w < 0` | `9d38405` |
| Frame docs (#7, partly) | Frames and sign conventions written down, using only what the team docs define. The gimbal tilt direction is now stated, and the open items are the servo direction and which servo is X or Y | `a8d7ee6`, `4a174fa` |
| Normalize (#8, partly) | `quaternion_norm`, `quaternion_normalize` | `b9dbb3e` |
| Quaternion helpers (#9) | multiply, conjugate, and `quaternion_to_rotation_vector` (the docs' phi formula) | `818f147`, `08add9e` |
| PID (#10) | Single-axis PID for the gimbal and a 3-axis PID with 3x3 matrix gains for torque. Integrators clamped, caller supplies the error rate | `8dcca75` |
| Include guard (#14) | Checked, already correct. No change needed | none |
| State input (#15) | `fc_state_t` (measured orientation, omega, position, velocity) replaces the estimator's own type, so the controller no longer depends on `eskf.h` | `a9cd632` |
| Gimbal conversions | `gimbal.h` and `gimbal.c`: the thrust direction for a pair of gimbal angles, and the angles for a desired thrust direction, with the sign convention. The limits are named in radians (`gimbal_max_x_rad`, `gimbal_max_y_rad`) | `4a174fa` |
| CI coverage | `pid.c`, `quaternion.c`, `flight_controller.c` and `gimbal.c` are compiled in the library build | `8dcca75`, `e5e5627`, `a9cd632`, `4a174fa` |
| Lint | clang-format fix for the super-linter | `4fd473c` |

A motor table and DShot encoder were written, then reverted (`35a90f9`, `e7c217b`). The code is still in the history.

**Known limits of the current state**
- Nothing calls the helpers, PIDs or gimbal conversions yet. `flight_controller_step` still outputs zeros.
- CI compiles the controls and math code but runs no tests. The host tests I wrote for the controller, PID, quaternion and gimbal code exist only on my machine.
- Flagged in the code, not fixed: a zeroed or forgotten config is accepted silently, the control law is not implemented, and the tests do not run in CI. Still open: `set_gains` does not validate the gains, and the gimbal output clamp is not implemented.

## Open issues

| Issue | Status | Waiting on |
|---|---|---|
| **#7** frames and gimbal signs | Docs written and corrected (`4a174fa`): the gimbal tilt direction is now stated. Frames, the thrust axis, which servo is X or Y and which motor is on top are still open | Controls team, firmware |
| **#8** renormalize after gyro update | Helper done; call goes in `eskf_predict`, which is an empty stub | ESKF owner |
| **#11** thrust to motor layer | On hold. Decided: throttle in %, one 2D table per motor indexed by thrust and roll torque | Firmware lead, controls, mechanical |
| **#12** `pos_ref` / `vel_ref` | Answered in part: the fields stay, as desired values hard-coded for the hop test. How a position or velocity error becomes a thrust demand is still undefined | Controls team |
| **#13** `fc_config_t` from CAD | On hold. No CAD values in the repo or docs, and no instance of the config exists. Only a gimbal limit estimate, about 18 degrees | Mechanical team, controls |
| **#16** Joseph-form update, EKF doc typos | On hold. The update step is an empty stub on the ESKF branch, so there is nothing to convert yet. Docs typo fixes are prepared locally but not submitted, and one typo (`u_k`) is unresolved | ESKF owner, docs author |

## Questions that need answers

**Controls team**
1. Frames: the world and body axes, the direction of gravity, which frame omega, the inertia and the thrust direction are in, and where torques are converted between frames. The ESKF branch's comments say North/East/Down and forward/right/down. That has +z pointing down, but the docs' thrust vector has a positive z component, so the thrust axis needs reconciling. The handwritten notes assume omega, the inertia and the gimbal are in the body frame and `q_err` is in the world frame, and cite the Rocket Proposal for rotating at the junctions. Can that be confirmed?
2. Units, and the physical servo direction and mounting. The docs' `t_hat` formula already implies that a positive `theta_x` tilts the thrust toward -y and a positive `theta_y` toward +x, in the frame `t_hat` is expressed in.
3. The docs list `MMOI` as a scalar, but the controller holds a 3x3 tensor. With a scalar, `tau_gyro = cross(MMOI*omega, omega)` is always zero. Is a tensor meant?
4. Where does the desired thrust magnitude come from? A fixed hover thrust, a vertical loop, or a mission input? Is an outer position loop planned (#12)?
5. Are the table inputs thrust in N and roll torque in N·m? Should `fc_command_t` change from `thrust_top` / `thrust_bottom` to a thrust magnitude and a roll torque?
6. The control loop rate, and whether `dt_s` is fixed or measured. Should a bad `dt_s` halt the flight computer, or should the controller hold the last command?
7. The PID gain values and `integral_max` limits, and what the gimbal derivative term should use.
8. The contents of `Rocket_Proposal.pdf`, which I could not read.

**Mechanical and simulation team**
1. Mass, inertia tensor, centre of mass and gimbal pivot from CAD. State the reference point, the axes and the CAD revision (#13).
2. Gimbal limits. The simulation used 14 degrees, but the gimbal is being redesigned.
3. The propeller characterization data, the table grid, and the max throttle per motor (#11).
4. Which of the two motors is the top one.

**Firmware lead**
1. Which ESC flies, and which protocol and speed does it use? The pin sheet shows two PWM lines plus a UART to each ESC board, which points at the custom ESC V1.0. The design notes say it is not ready, and the AM32 and DShot pages describe a commercial ESC.
2. Is changing the linker script and the CubeMX DMA setup acceptable? The CM7's only data RAM is the 128K at `0x20000000`, which DMA cannot reach, and no DMA is configured (#11).
3. What should the motors do on a fault, and what is the minimum throttle that keeps them spinning?
4. Where do the control task, its config and its gains queue live, and how do the M4 and M7 split the work?
5. Should host unit tests run in CI? Random121's branch already adds a `tests/unit` setup.

**ESKF owner** (`dev/13carpileup/eskf-architecture`)
1. The branch does not build with the project's flags: `math.c` includes its own `math.h` instead of the system one, so `sqrt` is undeclared, and `eskf.c` has conflicting types for `null_state` and `copy_config`.
2. `quat_multiplication` returns wrong x and z components. Tested against the Hamilton product, for a = (0.5, 0.5, -0.5, 0.5) and b = (0.1, 0.7, 0.3, -0.6) it gives x = 0.85 and z = 0.35, where the correct values are 0.55 and 0.25.
3. The branch duplicates the quaternion helpers in `quaternion.h`. The two should converge on one implementation.
4. `eskf_state_t` has no angular velocity, which the controller needs. How should omega reach the controller?
5. When will `eskf_predict` be implemented, so the normalize call can go in (#8)? Should it also normalize after the measurement update?
6. The ESKF overview page lists a barometer bias `beta_baro` in the state, but `eskf_state_t` has none and `ESKF_ERROR_STATE_SIZE` is 15. With the baro bias it would be 16. Which is intended?
7. `eskf_update_position` is where the Joseph-form covariance update and the Cholesky solve for the gain go (#16). The EKF derivation page already gives both (eq. 8a and Appendix B). CMSIS-DSP is not in the repo, and for the 3x3 position update a small hand-written Cholesky is enough.

**Random121** (`dev/random121/add-bdshot-lib`): the bidirectional DShot library there overlaps with #11. Is it the intended protocol layer?

**Docs author** (Kenneth Lew, the EKF pages in `UBC-Rocket/docs`)
1. The measurement model is defined as `h(x_k)` with no input (`h: R^m -> R^n`), but eqs. 4, D and E write `h(x_hat_{k|k-1}, u_k)`. Does `h` depend on `u`? If yes, the definition is wrong. If no, the `u_k` should go. This is the second typo named in #16, and I did not guess.
2. The ESKF overview writes `q <- q (x) Exp_q(omega_norm dt)`, where it defines `omega_nom`. This looks like a typo.

## Findings worth knowing

- **The docs do give the phi formula.** The architecture page has `phi = 2*atan(|v|/q0)*v_hat`. An earlier note of mine saying it was missing was wrong. It is now implemented as `quaternion_to_rotation_vector`.
- **RPM versus throttle.** The architecture page says the tables output RPM, and the newer characterization page says throttle in percent. We chose throttle.
- **The DShot page is a stub.** It holds one outside link and no design.
- **Hardware.** The motor outputs are `TIM1` CH1 (PE9, `ESC1_PWM`) and CH2 (PE11, `ESC2_PWM`) on the CM7, with a 240 MHz timer clock. `TIM1` is initialized in `main.c` but never started.
- **Propellant does not apply.** The TVR is electric, so mass is constant in flight. The config is most likely set once per flight.
- **EKF page typos, fixed locally only.** Seven slips on the "EKF Full Derivation" page are fixed in one commit (`039cd30`, branch `fix/ekf-derivation-typos`) in a scratch clone of `UBC-Rocket/docs`. They are the `P_{k|k}` versus `P_{k|k-1}` subscripts in prediction step 2, a reversed sentence about the innovation, "equation F and G" (should be F and equation 7), the `delta_{k|k}` subscript in the Joseph derivation, `k_k` for `K_k`, "semi-definite" for "definite", and a missing transpose in Appendix A. Nothing was pushed and no PR was opened. The clone is deleted with the session, so the commit would need exporting as a patch to keep it.
- **The EKF page already supports #16.** It boxes the Joseph form (eq. 8a) beside the simplified update (8b), and Appendix A derives 8b from 8a only for the optimal gain, which is why Joseph form is safer in float.

## Cross-check with the handwritten notes (16 pages)

Checked on 22 Nov against the repo and this report. Most of the notes match. These are the differences.

**New information that corrects earlier statements**
- **The gimbal tilt direction is implied by the docs.** The architecture page's `t_hat = [sin(theta_y), -sin(theta_x)*cos(theta_y), cos(theta_x)*cos(theta_y)]` means a positive `theta_x` tilts the thrust toward -y and a positive `theta_y` toward +x. An earlier version of this report, issue #7 and the "not defined" list in the `flight_controller.h` comment all said this was undefined. The header is now corrected (`4a174fa`). What is still open is the physical servo direction and mounting, which a bench check with the thrust visible would settle.
- **A thrust-axis conflict.** The notes infer +z along the thrust axis, because the `atan(t_y/t_z)` formula needs it. The ESKF branch's body frame is forward/right/down, where +z points down. If both describe the same body frame, the thrust would point along -z.
- **Rotating at the junctions.** The notes say the Rocket Proposal PDF says to rotate frames at the junctions. This report could not read that PDF, so it said the docs do not say where frames are converted. If the PDF does say that, #7's frame-conversion question is partly answered, and the `flight_controller.h` comment needs updating.

**Notes that are out of date**
- p.4 lists `prev_error` state in the handle. Since #10 the handle holds only the PID integrators, and the derivative comes from a caller-supplied error rate.
- p.7 records the threading brainstorm (mutex, queue of 1), not the final design, which is a lock-free pending-gains buffer.
- p.9 says two static asserts guard the gains buffer. These were replaced by a runtime check and `FC_FAULT_BAD_GAINS_LAYOUT` (`5100a0d`). The missed-tick flag is `FC_FAULT_MISSED_TICK`, not `FC_MISSED_TICK` (p.10).
- p.10's "no lower bound on `dt`" is moot for the derivative, which no longer divides by `dt`. "Unverified since `eskf.h` is missing" no longer holds: `pid.c`, `quaternion.c`, `flight_controller.c` and `gimbal.c` all build in CI.
- p.11 reads "w = 0 is 130 degrees". It is 180 degrees. The handwriting is unclear.

**Notes that describe something the code does not do yet**
- **Gain validation (p.6, step 4).** `flight_controller_set_gains` only checks for NULL pointers. There is no finite or limit check on the gains.
- **Gimbal output clamp (p.14).** Only the integrator clamp exists. Clamping the gimbal output to `gimbal_max_x_rad` / `gimbal_max_y_rad` belongs in the control law, which is not written.
- **Radians (p.1, ticked).** Fixed in `4a174fa`: the limits are now named `gimbal_max_x_rad` and `gimbal_max_y_rad`.

## Cross-check with the Notion export

The export is one page, "Ulysses Firmware for Flight Controller", about 1,000 lines. The code listings match what is committed, and the notes for #11 to #16 match the comments posted on the issues. These are the differences.

**Minus signs lost in the copy-paste**
- Lines 271 and 394 read "`q` and `q` give the same result" and "`q` versus `q`". They should be `q` and `-q`.
- Line 666 says the caller passes `error_rate` "for example `omega`". It should be `-omega`, and the sign matters for the derivative term.
- Lines 390, 395 and 675 show `Wall` where the flag is `-Wall`.

**Stale sections**
- The #1 and #2 struct listings still show the `prev_err` arrays and lack the atomic `faults` field and the gains buffer. The #2 doc comment says "previous-error state".
- The #3 table still lists `prev_err`, and its `init` listing lacks the `faults` setup and the gains-layout check.
- The #4 text mentions two compile-time checks. These became a runtime check plus `FC_FAULT_BAD_GAINS_LAYOUT`.

**Thin sections**
- #5 is one line. #7 holds only the issue text, with no findings or frame docs. #4 stops at "In step:".
- Nothing records the gimbal sign finding from the handwritten notes: a positive `theta_x` tilts the thrust toward -y and a positive `theta_y` toward +x.

**The pasted report at the bottom is out of order and lost its content**
- The title is at the very end. The team question lists, Findings, Next steps and the cross-check are empty bullets or numbered items, and "Questions that need answers" holds the status table instead of the questions.
- This report file is complete and in order. Importing the file into Notion as Markdown, rather than pasting rendered text, should keep the question lists.

## Suggested next steps

1. Send the question lists above to each team. The controls team's answers unblock the most: #7, #11, #12 and the control law.
2. Done: PR #17 is merged into `main` (`23f7b1b`). Its merge commit preserves Kenneth Lew's three commits.
3. When the ESKF branch merges, resume #8 and write the adapter from its output to `fc_state_t`.
4. Write the control law once the frames are settled: `q_err`, `phi`, the PID calls, then the thrust and gimbal formulas.
5. Decide what to do with the docs typo fixes: open a PR to `UBC-Rocket/docs`, or export the commit as a patch for Kenneth Lew. Ask him about the `u_k` typo first.
6. Close the gaps the handwritten notes exposed that are still open: add finite and limit checks to `flight_controller_set_gains`, and implement the gimbal output clamp in the control law. The `_rad` naming and the corrected frame notes are done (`4a174fa`).
7. Share the Rocket Proposal PDF, or the page about rotating at the junctions, so the frame question in #7 can be settled.
8. Fix the Notion page. The new documentation page (`ulysses-flight-controller-documentation.md`) has the minus signs restored (`-q`, `-omega`, `-Wall`), the #1 to #4 sections updated to the current code, and #5 and #7 filled in. Import it, and replace the broken pasted report with this file.
