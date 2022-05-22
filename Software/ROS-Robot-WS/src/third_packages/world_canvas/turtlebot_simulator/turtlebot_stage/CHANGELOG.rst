^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot_stage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2017-09-18)
------------------
  Fixed include path for turtlebot_navigation/launch/includes/amcl/amcl.launch.xml. https://github.com/turtlebot/turtlebot_apps/commit/cc2671666e8529677e6757fe78de4ff625f946e5
* Contributors: Clyde McQueen

2.2.2 (2015-09-16)
------------------
* view frame is now base_link closes `#51 <https://github.com/turtlebot/turtlebot_simulator/issues/51>`_
* Contributors: Jihoon Lee

2.2.1 (2015-08-07)
------------------
* turtlebot_stage: add turtlebot_navigation dependency
* Contributors: Gaël Ecorchard

2.2.0 (2014-12-30)
------------------
* Adds border extension to map to fix map scaling / position in stage
* Added changes in the stage launch file undone before because of the merge
* Merged
* Moved env fold inside stage
* Revert "Stage launch use env variables"
* Small fix for wrong file name of env script
* Added support for map and world file through env variables
* remove annotation and param install rule
* updated rviz configuration for dwa.
* add the trajectory cloud to stage's rviz view.
* stage only permits 'square' robots, so make sure our square fits within
  the turtlebot defined circle otherwise it collides when it normally
  wouldn't.
* redirect all params and launchers at the real turtlebot configuration files and cleanup, `#21 <https://github.com/turtlebot/turtlebot_simulator/issues/21>`_.
* remove unused plugins and add the cost cloud.
* add turtlebot_stage
* Contributors: Alexander Reimann, Daniel Stonier, Jihoon Lee

* Adds border extension to map to fix map scaling / position in stage
* Added changes in the stage launch file undone before because of the merge
* Merged
* Moved env fold inside stage
* Revert "Stage launch use env variables"
* Small fix for wrong file name of env script
* Added support for map and world file through env variables
* remove annotation and param install rule
* updated rviz configuration for dwa.
* add the trajectory cloud to stage's rviz view.
* stage only permits 'square' robots, so make sure our square fits within
  the turtlebot defined circle otherwise it collides when it normally
  wouldn't.
* redirect all params and launchers at the real turtlebot configuration files and cleanup, `#21 <https://github.com/turtlebot/turtlebot_simulator/issues/21>`_.
* remove unused plugins and add the cost cloud.
* add turtlebot_stage
* Contributors: Alexander Reimann, Daniel Stonier, Jihoon Lee

2.1.1 (2013-10-14)
------------------

2.1.0 (2013-09-02)
------------------

2.0.0 (2013-08-12)
------------------

1.9.1 (2013-01-02)
------------------

1.9.0 (2012-12-22)
------------------
