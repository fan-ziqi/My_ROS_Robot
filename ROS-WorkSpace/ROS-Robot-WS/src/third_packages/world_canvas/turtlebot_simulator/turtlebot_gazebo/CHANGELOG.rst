^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2017-09-18)
------------------
* Fix changing amcl.launch.xml and gmapping.launch.xml locations under turtlebot_navigation
* Contributors: Mohamed Al Zaiady, Mohamed Elzaiady, Tully Foote, mzaiady

2.2.2 (2015-09-16)
------------------

2.2.1 (2015-08-07)
------------------
* Enable to run Gazebo w/o GUI.
* Contributors: Isaac IY Saito

2.2.0 (2014-12-30)
------------------
* use env hook to configure gazebo world and map fixes `#40 <https://github.com/turtlebot/turtlebot_simulator/issues/40>`_
* now it uses args to load world. and corridor world is added. `#39 <https://github.com/turtlebot/turtlebot_simulator/issues/39>`_
* disable create gazebo plugin
* Add turtlebot_navigation to turtlebot_gazebo depends
  gmapping_demo.launch depends on it.
* Contributors: Jihoon Lee, Jochen Sprickerhof

2.1.1 (2013-10-14)
------------------
* Rename cmd_vel_mux as yocs_cmd_vel_mux.

2.1.0 (2013-08-30)
------------------
* Add navigation demos on Gazebo on a playground world.
* Add bugtracker and repo info URLs.
* Add cmd_vel_mux for create and roomba.
* Do not use robot_pose_ekf for kobuki base.

2.0.0 (2013-07-16)
------------------

* Migrated to use stand-alone Gazebo installation
* All packages have been catkinized
