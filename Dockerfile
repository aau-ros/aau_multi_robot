FROM ros:indigo

# place here your application's setup specifics

CMD [ "roslaunch", "multi_robot_simulation master_ext.launch" ]
