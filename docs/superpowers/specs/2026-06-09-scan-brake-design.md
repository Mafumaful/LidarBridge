# Yuyi Controller Scan Brake Design

## Summary

Add a configurable scan-based braking layer to `yuyi_controller` so the controller can stop and resume based on `/scan` obstacle proximity. The controller will divide the laser scan into eight fixed sectors around the vehicle, evaluate the nearest valid range in each enabled sector, and gate path-following motion using per-sector braking thresholds.

This feature must work at runtime through ROS 2 dynamic parameters. When braking is triggered, the vehicle must decelerate to a stop using the configured braking deceleration instead of forcing an immediate zero command. When the environment becomes safe again, the controller must resume motion while respecting the configured acceleration limit.

## Goals

- Add optional `/scan`-based motion gating to `yuyi_controller`.
- Split the laser scan into eight fixed vehicle-relative sectors.
- Allow each sector to be enabled or disabled independently.
- Allow each sector to define its own braking distance threshold.
- Support runtime adjustment through ROS 2 parameters without restarting the node.
- Preserve existing pure pursuit behavior when scan braking is disabled.

## Non-Goals

- No separate safety node or `/cmd_vel` relay layer.
- No adaptive sector width or arbitrary user-defined angular ranges.
- No reverse-specific behavior beyond the same sector threshold evaluation.
- No change to path loading, TF lookup, or pure pursuit geometry outside the scan brake gate.

## User-Facing Behavior

### Feature Switch

- `use_scan_brake=false`
  - The controller ignores `/scan`.
  - Startup and motion behavior remain unchanged from the current implementation.
- `use_scan_brake=true`
  - The controller subscribes to `/scan` and requires scan data before motion is allowed.
  - If no scan has been received yet, the vehicle remains stopped.

### Braking Rule

Each control cycle:

1. Evaluate the latest valid scan.
2. Compute the nearest valid range for each of the eight sectors.
3. For every enabled sector, compare the nearest range against that sector's braking distance.
4. If any enabled sector is unsafe, force the controller target speed to `0.0`.
5. Use the existing deceleration-limited speed ramp so the vehicle brakes smoothly according to `max_deceleration_mps2`.

This means obstacle braking is not an instantaneous hard stop unless the current deceleration settings make it so.

### Resume Rule

When all enabled sectors are safe again:

- The controller returns to its normal pure pursuit target speed calculation.
- The existing acceleration-limited ramp continues to apply, so motion resumes gradually according to `max_acceleration_mps2`.

### Scan Availability Rule

If `use_scan_brake=true`:

- Before the first `/scan` message arrives, the controller must not move.
- If scan messages were received previously but the latest one is older than the configured timeout, the controller must stop.
- If an enabled sector has no valid measurement in the latest scan, that sector is treated as unsafe.
- If a direction is not covered by the physical sensor, that sector should be configured with `enabled=false`.

## Sector Model

The laser scan is partitioned into eight fixed sectors in the vehicle frame:

- `front`: `[-22.5 deg, 22.5 deg)`
- `left_front`: `[22.5 deg, 67.5 deg)`
- `left`: `[67.5 deg, 112.5 deg)`
- `left_rear`: `[112.5 deg, 157.5 deg)`
- `rear`: `[157.5 deg, 180 deg]` and `[-180 deg, -157.5 deg)`
- `right_rear`: `[-157.5 deg, -112.5 deg)`
- `right`: `[-112.5 deg, -67.5 deg)`
- `right_front`: `[-67.5 deg, -22.5 deg)`

Each sector uses the nearest valid range sample inside its angular span as the braking reference.

A valid range sample is one that:

- falls within the sector's angular span
- is finite
- is within the scan message's valid range interval

## Parameters

### New Parameters

Global parameters:

- `use_scan_brake` (`bool`, default `false`)
- `scan_topic` (`string`, default `"/scan"`)
- `scan_max_age_sec` (`double`, default `0.5`)

Per-sector parameters:

- `scan_brake.front.enabled`
- `scan_brake.front.brake_distance_m`
- `scan_brake.left_front.enabled`
- `scan_brake.left_front.brake_distance_m`
- `scan_brake.left.enabled`
- `scan_brake.left.brake_distance_m`
- `scan_brake.left_rear.enabled`
- `scan_brake.left_rear.brake_distance_m`
- `scan_brake.rear.enabled`
- `scan_brake.rear.brake_distance_m`
- `scan_brake.right_rear.enabled`
- `scan_brake.right_rear.brake_distance_m`
- `scan_brake.right.enabled`
- `scan_brake.right.brake_distance_m`
- `scan_brake.right_front.enabled`
- `scan_brake.right_front.brake_distance_m`

Suggested defaults:

- all sectors `enabled=false`
- all sector `brake_distance_m=0.5`

This keeps current behavior unchanged until the feature is intentionally enabled and configured.

### Existing Parameters Reused

The following existing parameters remain authoritative:

- `max_acceleration_mps2`
- `max_deceleration_mps2`

They define the motion envelope for both obstacle-triggered stopping and safe-state resumption.

### Dynamic Parameter Behavior

These parameters must be dynamically adjustable at runtime:

- `use_scan_brake`
- `scan_max_age_sec`
- every `scan_brake.*.enabled`
- every `scan_brake.*.brake_distance_m`
- `max_acceleration_mps2`
- `max_deceleration_mps2`

These parameters should remain static after node startup:

- `scan_topic`
- frame IDs
- publisher topic names
- `path_file`

Validation rules:

- `scan_max_age_sec > 0.0`
- each `brake_distance_m >= 0.0`
- `max_acceleration_mps2 > 0.0`
- `max_deceleration_mps2 > 0.0`

Invalid runtime parameter updates must be rejected.

## Internal Design

### Node Integration

The scan brake feature stays inside `YuyiControllerNode`. No new ROS node is introduced.

The node gains:

- a `/scan` subscription
- state storage for the latest scan message and its receipt time
- sector configuration storage
- a dynamic parameter callback for scan brake settings

### Suggested Internal Structures

Use small internal structures to keep the control loop readable:

- `SectorConfig`
  - `enabled`
  - `brake_distance_m`
- `SectorObservation`
  - `has_valid_range`
  - `nearest_range_m`
- `ScanBrakeEvaluation`
  - per-sector observations
  - `has_fresh_scan`
  - `should_brake`
  - optional textual reason for logging

These can remain local to `yuyi_controller_node.cpp` unless later reuse justifies moving them to headers.

### Control Flow

The controller loop priority should be:

1. TF unavailable: publish stop.
2. Scan brake enabled and scan unavailable, stale, or unsafe: target speed becomes `0.0`.
3. Goal-stop condition reached: target speed becomes `0.0`.
4. Otherwise: compute pure pursuit target speed normally.
5. Apply the existing acceleration and deceleration ramp using `move_towards`.

This keeps all stop conditions consistent and reuses the controller's existing speed shaping logic.

### Scan Processing

The scan callback stores the latest scan message. Sector evaluation happens from the cached latest scan during `control_step()`, not inside the callback. This keeps braking decisions synchronized with the controller loop and avoids splitting core decision logic across threads of execution.

Processing steps:

1. Iterate every scan sample angle.
2. Normalize the sample angle into `[-pi, pi]`.
3. Map the sample into one of the eight sectors.
4. Ignore invalid samples.
5. Track the minimum valid range for each sector.
6. Mark sectors without any valid range as invalid for this frame.

## Logging and Debugging

The controller should log scan brake state with throttling:

- waiting for first scan
- stale scan timeout
- sector-triggered braking with sector name and measured range
- scan brake released and motion allowed again

This should be informative enough for field debugging without flooding the console.

## Testing Strategy

### Unit Tests

Add focused tests for:

- sector angle classification
- minimum-range extraction per sector
- no-motion behavior before first scan when `use_scan_brake=true`
- unsafe result when an enabled sector has no valid sample
- braking decision when measured range is below threshold
- safe result when all enabled sectors exceed thresholds
- runtime parameter updates taking effect immediately

### Node-Level Tests

Add or extend controller-level tests to cover:

- node startup with scan braking disabled preserves current behavior
- node startup with scan braking enabled and no scan keeps command output at zero
- safe scan allows nonzero motion commands once TF and path are available
- unsafe scan drives commanded speed toward zero
- stale scan re-triggers stopping after timeout

Tests should follow the package's current lightweight C++ test style unless a more realistic ROS integration harness becomes necessary.

## Risks and Tradeoffs

- Keeping the feature in the existing node minimizes deployment change, but the source file will grow further unless helper functions are kept disciplined.
- Treating missing sector data as unsafe is conservative and may require users to disable sectors outside real sensor coverage.
- Scan timeout behavior adds another stop reason, so logs need to make the active reason obvious.

## Rollout Notes

- Update `config/yuyi_controller_params.yaml` with safe defaults.
- Update the package README with the new parameters and runtime tuning examples.
- Preserve backward compatibility by leaving scan braking disabled by default.
