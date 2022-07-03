ros2 service call \
/apply_joint_effort \
gazebo_msgs/srv/ApplyJointEffort \
'{ joint_name: "joint2", effort: -1.0, start_time: {sec: 0, nanosec: 0}, duration: {sec: 2000, nanosec: 0} }' 

ros2 service call \
/apply_link_wrench gazebo_msgs/srv/ApplyLinkWrench \
'{link_name: "link2", reference_frame: "", reference_point: { x: 100, y: 0, z: 0 }, wrench: { force: { x: 10, y: 0, z: 0 }, torque: { x: 0, y: 0, z: 0 } }, start_time: {sec: 0, nanosec: 0}, duration: {sec: -1, nanosec: 0} }'

ros2 service call \
/clear_joint_efforts gazebo_msgs/srv/JointRequest \
'{joint_name: "joint2"}'

ros2 service call \
/clear_link_wrenches gazebo_msgs/srv/LinkRequest \
'{link_name: "link2"}'