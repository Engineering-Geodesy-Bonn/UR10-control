# DOCKER

""" sudo docker run --rm -it --network host ur_modern_driver:melodic """
""" roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=10.1.1.2 """
""" roslaunch ur_modern_driver ur_bringup_rviz.launch robot_model:=ur10 robot_ip:=10.1.1.2 """

# Move ARM: 

sudo docker ps # get container ID
sudo docker exec -it  4ed9a653c719 bash
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0, -1.57, 0, 0, 0, 0], a=1.2, v=1.2)'"


# Drive to list of points read from csv data
rosrun ur_modern_driver move_to_xyz.py

# Bring robot in initial position
rostopic pub -1 /ur_driver/URScript std_msgs/String "data: 'movej([0.0, -1.5708, 0.0, -1.5708, 0.0, 3.1415], a=0.5, v=0.5)'"