# Astribot S1 arm control boundary

Quest3 arm teleoperation uses only the official Astribot Python client. The control PC constructs `Astribot(freq=100.0)`, reads desired/current Cartesian poses in the chassis frame, and sends the enabled arm targets together with one `set_cartesian_pose(..., control_way="filter", use_wbc=False)` call per control cycle.

The following Orin topics are feedback-only and must never have publishers in this project:

- `/astribot_arm_left/endpoint_current_states`
- `/astribot_arm_right/endpoint_current_states`
- `/astribot_arm_left/endpoint_desired_states`
- `/astribot_arm_right/endpoint_desired_states`

The following topics are internal-only joint commands owned by the existing Astribot driver/robotics stack and must never be published by teleoperation code:

- `/astribot_arm_left/joint_space_command`
- `/astribot_arm_right/joint_space_command`

The observed baseline is approximately 250 Hz for endpoint current and desired feedback. Deployment uses `ROS_DOMAIN_ID=25`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and the Fast DDS `192.168.0.x` interface allowlist. Dry-run mode does not import or initialize the Astribot SDK. Real initialization refuses operation when the SDK reports that control rights are unavailable, which catches a competing SDK/control client at the supported API boundary.

M7-M10 robot validation is deliberately deferred. Do not infer safe hold behavior for SDK exit, process death, or network loss until it has been measured with the physical E-stop operator present.
