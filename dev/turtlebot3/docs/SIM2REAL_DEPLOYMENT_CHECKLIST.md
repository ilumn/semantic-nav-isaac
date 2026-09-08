# Sim2Real Deployment Checklist

> Historical completion record. Gazebo-specific items describe the predecessor
> implementation; see `docs/ISAAC_SIM_PORT.md` for the supported simulator.

## Planning And Docs

- [x] Save detailed sim2real deployment plan.
- [x] Save corresponding implementation checklist.
- [x] Document final real-robot operator runbook.
- [x] Document onboard dependency provisioning for Jazzy on Ubuntu 24.04.
- [x] Document robot bench-test and floor-test procedures.

## Shared Launch Architecture

- [x] Add shared `semantic_nav_stack.launch.py` for the ROS stack.
- [x] Refactor simulation launch to wrap Gazebo/bridges around the shared stack.
- [x] Add `real_semantic_nav.launch.py` for real robot deployment.
- [x] Ensure real launch never starts Gazebo, `ros_gz_bridge`, or `ros_gz_image`.
- [x] Ensure real launch defaults to `use_sim_time:=false`.
- [x] Ensure sim launch defaults to `use_sim_time:=true`.

## Runtime Profiles And Config

- [x] Add sim profile config.
- [x] Add real Waffle Pi profile config.
- [x] Install profile config files through the coordinator package.
- [x] Add launch arguments for map, costmap, odom, camera, scan, query, and refiner settings.
- [x] Add real Nav2 parameter file.
- [x] Add real SLAM Toolbox parameter file.

## Real Robot Safety

- [x] Add explicit motion arm/disarm service.
- [x] Default real robot runtime to unarmed.
- [x] Cancel active Nav2 goals on disarm.
- [x] Block frontier exploration until armed.
- [x] Block semantic navigation goals until armed.
- [x] Publish clear coordinator readiness and arm state.

## Readiness Checks

- [x] Verify `/scan` is present and fresh.
- [x] Verify `/odom` is present and fresh.
- [x] Verify `/camera/image_raw` is present and fresh.
- [x] Verify `/camera/camera_info` is present and fresh.
- [x] Verify `/tf` and `/tf_static` are present.
- [x] Verify `map -> odom -> base_link -> camera` TF chain before arming.
- [x] Surface readiness failures in `/coordinator_node/status`.

## Frontier Exploration

- [x] Validate exploration still moves in simulation through the shared stack.
- [x] Fix current `safe=0 rejected=1` behavior in `detector_test.world`.
- [x] Confirm `/frontiers` publishes valid candidates.
- [x] Confirm `goal_assignment_node` sends Nav2 goals after arm.
- [x] Confirm `/cmd_vel` publishes during exploration.
- [x] Confirm warmup stops publishing after completion.

## Semantic Navigation

- [ ] Confirm detector runs from the real camera topic.
- [ ] Confirm localizer fuses real camera detections with real LDS scan.
- [x] Confirm native semantic map publishes `/semantic_map/state`.
- [x] Confirm semantic query uses `semantic_map_state` by default.
- [x] Confirm nav adapter emits map-frame standoff goals.
- [x] Confirm semantic query navigation works after arm.

## Refiner And COLMAP Enrichment

- [x] Keep refiner enabled onboard by default.
- [x] Make refiner degradation non-blocking.
- [ ] Confirm bundle writing works on the real robot filesystem.
- [ ] Confirm `semantic-nav-memory` dependencies are available.
- [x] Confirm COLMAP point clouds publish when jobs succeed in the ROS overlay path.
- [x] Confirm rejected or weak alignments do not publish stale overlays.

Notes:

- Real camera detector/localizer confirmation, real robot bundle filesystem confirmation, and bench/floor tests require access to the physical TurtleBot3.
- Full `semantic-nav-memory` worker execution is blocked in this environment until `colmap`, `pycolmap`, and `ultralytics` are installed. The accepted-job ROS pointcloud publisher path was validated with a synthetic completed refinement job.

## Validation

- [x] Run Python launch syntax checks.
- [x] Run `ros2 launch ... --show-args` for sim and real launch files.
- [x] Run focused `colcon build`.
- [x] Run short simulation smoke test.
- [ ] Run real robot bench test.
- [ ] Run real robot bounded floor test.
