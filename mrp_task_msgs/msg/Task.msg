uint8 task_group_id
uint8 task_id
geometry_msgs/PoseStamped location     # Location where the task should be executed

# We need a mechanism to describe how a task should be executed
# For now it is simply set as a target that the robot needs to go to