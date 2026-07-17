# ADR-007: Contact Force Source of Truth

## Status
Accepted (documenting existing implementation)

## Context
The SimEnv simulation uses Gazebo contact sensors to provide foot-ground contact force data for the Trotting gait controller. The contact force data flows through a multi-stage pipeline from Gazebo physics to the FSM readiness gate.

## Decision

### Source of Truth
The **sole** source of truth for simulated foot contact force is the Gazebo `ContactSensor` → `UnitreeFootContactPlugin` → ROS `/visual/{leg}_foot_contact/the_force` topic chain.

### Force Computation
- The `UnitreeFootContactPlugin` averages all contact point forces from `body_1_wrench().force()` in the foot link's local frame.
- `IOROS::updateFootForce()` computes the Euclidean norm `sqrt(x² + y² + z²)` and stores it as a scalar magnitude in Newtons.
- This magnitude is **always positive**, losing directional information but providing a robust contact/non-contact signal.

### Freshness
- Contact data freshness is gated by wall-clock time: `footForceValid[i] = (now - last_callback_time) <= 1 second`.
- This is a **staleness guard**, not a precision timestamp. Its purpose is to detect when the contact sensor has stopped publishing.
- During Gazebo pause, wall time continues but sim time stops. The FSM's `updateControlTime()` prevents false staleness during pause by not evaluating readiness.

### Alternate Sources (Rejected)
- **Motor current/torque estimation**: Not available in simulation (torque comes from PD controller, not physical load).
- **Kinematic ground penetration**: Unreliable; Gazebo's ODE solver resolves penetration instantly.
- **Ground truth plugin**: Would require separate infrastructure; existing contact sensors suffice.

## Consequences
- The contact force magnitude cannot distinguish push from pull (both produce positive values).
- A 1-second wall-time staleness window is conservative; slow simulations (RTF < 0.01) could theoretically false-trigger, but such low RTF is outside normal operating range.
- Four independent WrenchStamped topics mean each foot updates independently; torn reads across feet are possible but acceptable for the readiness gate (all-or-nothing check).
