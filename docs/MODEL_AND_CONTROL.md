# Model and control notes

## Coordinates and geometry

World: x/y horizontal, z up; lengths in m, angles rad, mass kg, time s, torque Nm. Base pose is [x, y, yaw]. The arm origin is at floor height at the base center. First-link height is 0.62 m, upper and lower links are 0.62 and 0.52 m, and tool length is 0.14 m. The joint axes in successive local frames are Z, Y, Y, X, Y, X. The last three axes intersect at the wrist.

For joint i, R_i = R_(i-1) Rot(axis_i,q_i), and p_(i+1) = p_i + R_i offset_i. The tool pose is [R_6,p_7]. This explicitly specified geometry makes the closed-form IK reproducible; it is not an identified commercial robot.

## Closed-form IK and Jacobian

Subtract the oriented tool offset to obtain wrist center w. With rho = hypot(w_x,w_y), z = w_z-h:

- q1 = atan2(w_y,w_x)
- cos(q3) = (rho²+z²-L1²-L2²)/(2 L1 L2)
- q2 = atan2(-z,rho) - atan2(L2 sin(q3), L1+L2 cos(q3))

Both elbow branches and two X-Y-X wrist branches are checked. Select the joint-limit-feasible branch nearest the previous command. Reachability is rejected before acos; the resulting pose is checked numerically. The current solver uses the outward radial shoulder branch; it is not an exhaustive enumeration of all industrial-arm IK solutions.

Each revolute joint contributes Jv_i = axis_world_i × (p_tcp-p_i), Jw_i = axis_world_i. The smallest singular value identifies near-singular configurations, but this mixed translational/rotational Jacobian is not dimensionless. Do not interpret its numerical value as a universal safety threshold.

## Newton-Euler model

Each link has a specified mass, center of mass and positive isotropic inertia approximation. Rotor inertia and linear joint friction are included. The forward recursion computes world-frame angular velocity, angular acceleration, joint-origin acceleration and COM acceleration. The backward recursion accumulates link force and moment about each joint. A point payload at the TCP contributes inertial and gravity force.

The plant equation used in controller experiments is:

M(q) qdd + h(q,qd) = tau + d(t)

where h includes gravity, Coriolis/centrifugal terms and viscous friction. Columns of M are obtained by inverse dynamics at unit joint acceleration with zero velocity and gravity. Tests compare gravity torque with a finite-difference potential-energy gradient and verify symmetry and positive definiteness of M.

Joint limits are ±pi. Torque limits are [40,65,45,12,12,8] Nm. These are synthetic design parameters, not manufacturer limits. Coupling from an accelerating mobile base into arm dynamics is omitted; manipulation occurs while the base is stationary. Navigation assumes a stowed arm and an ideal no-slip base.

## References and feedback

The quintic timing function is s(u)=10u³-15u⁴+6u⁵, u=t/T. Its first two derivatives vanish at both endpoints. Joint paths interpolate q0 to q1. Cartesian approach/retract paths interpolate TCP position with fixed downward orientation and solve IK at each time step.

Mission animation integrates an acceleration-command joint servo: qdd_cmd = qdd_ref + Kp(q_ref-q) + Kd(qd_ref-qd). It checks required Newton-Euler torque, limits acceleration/velocity and checks final target error. This is an idealized inner joint servo; the mission alone does not validate torque tracking with model mismatch.

The independent torque-driven experiments compare:

1. PD + gravity: tau = Kp_tau e + Kd_tau edot + g_hat(q).
2. Computed torque: tau = ID_hat(q,qd,qdd_ref+Kp e+Kd edot).

Both torques are clipped to identical limits. The true plant may have different link masses/inertias. Semi-implicit Euler integration uses dt=0.005 s. Computed torque gives the intended linear error dynamics only with an accurate model and no saturation; robust performance must be read from measurements rather than assumed.

The RMS metric pools all joints and time samples (rad); peak error is maximum absolute joint error; effort is integral of sum(tau_i²) dt (Nm² s). Saturation fraction counts saturated joint-samples. Each controller receives the same random noise seed in a matched scenario. The combined stress case adds 2 Nm to shoulder joint 2 between 2 and 2.5 s.

## Base and safety

xdot=v cos(theta), ydot=v sin(theta), thetadot=omega. Ideal wheel rates are (v±track*omega/2)/wheel_radius. Four rendered wheels are treated as two commanded sides (skid-steer approximation). Closed-loop heading feedback aims toward a lookahead sample on a checked A* path; velocity drops for heading error and goal proximity. Docking regulates final yaw after position capture.

LiDAR casts rays against circular obstacle footprints and rectangular world walls. It includes Gaussian range noise. The stop field uses margin + |v| reaction_time + v²/(2 brake_deceleration). A crossing predictor uses the scripted AGV velocity; this is stronger information than a real unknown moving obstacle would provide. A ground-truth candidate-pose guard is a final simulation safeguard. Reported safety is therefore specific to this modeled experiment, not a general collision-avoidance guarantee.

## Contact, inspection and collision limitations

A block becomes rigidly attached when TCP position is within 15 mm. No contact force, friction, grasp stability or falling-block physics. Inspection is a pinhole projection of known centroids (f=310 pixels, 640×480 image), plus geometric stack alignment checks. No image recognition, camera noise or occlusion reasoning.

Collision monitoring includes a conservative circular base, world bounds, upright obstacle cylinders and sampled arm-link capsules against obstacles/AGV/floor. It excludes full robot self-collision, exact boxes, block-block/gripper contact and continuous swept-volume proofs. The collision count is violating simulation samples, not distinct impact events.

## Extended route and inspection revision

The route planner retains mandatory waypoints to lengthen travel through the workcell. Each leg uses eight-connected A* with both grid occupancy and continuous segment clearance checks. Quadratic corner blends are accepted only after collision checks; an unsafe blend is reduced or falls back to its corner. The follower searches locally forward along the path to avoid jumping to a nearby later route section.

Two clear transverse AGV lanes are selected per long leg. The crossing supervisor explicitly commands zero forward and angular velocity, with bounded braking, from activation until the AGV finishes traversing its lane. This is a scripted workcell interlock, not a perception-only prediction guarantee. AGV repositioning between lanes is abstracted as staging.

After both releases, the robot holds the wrist camera pose for six seconds. The green triangle/line visualizes a synthetic sweep across stack height; this does not add real image recognition, depth sensing or defect classification to the geometric inspection model.
