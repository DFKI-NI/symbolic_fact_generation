cmake_minimum_required(VERSION 3.0.2)
project(symbolic_fact_generation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Fact.msg
  Facts.msg
  ChangedFacts.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package()

# Testing
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_symbolic_fact_generation_lib.py)
  catkin_add_nosetests(test/test_symbolic_fact_generation_collision.py)
  catkin_add_nosetests(test/test_symbolic_fact_generation_on_generator.py)
  catkin_add_nosetests(test/test_gripper_facts_generator.py)
endif()
