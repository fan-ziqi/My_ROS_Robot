^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package warehouse_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.2 (2018-12-07)
------------------
* Fix various smaller issues. (`#41 <https://github.com/ros-planning/warehouse_ros/issues/41>`_)
  * fix guard name
  * virtual destructor for abstract class
  * use managed pointers - createUniqueInstance()
  * switch to C++11
  * clang-tidy modernize-use-override
* Contributors: Robert Haschke

0.9.1 (2018-10-17)
------------------
* fix missing return value (`#40 <https://github.com/ros-planning/warehouse_ros/issues/40>`_)
* update include statements to use new pluginlib and class_loader headers (`#38 <https://github.com/ros-planning/warehouse_ros/issues/38>`_)
* Contributors: Mikael Arguedas, Robert Haschke

0.9.0 (2016-06-20)
------------------
* [fix] Omit dependency on mongo (and replace with pluginlib) `#32 <https://github.com/ros-planning/warehouse_ros/issues/22>`_
* [fix] Specifically including a header that seems to be required from Ubuntu Xenial.
* [sys] Ensure headers and libraries are present for downstream pkgs `#17 <https://github.com/ros-planning/warehouse_ros/issues/17>`_
* [sys] Update CI config to test Jade and Kinetic `#30 <https://github.com/ros-planning/warehouse_ros/issues/30>`_
* [sys] Add rostest file and configs.
* Contributors: Connor Brew, Dave Coleman, Ioan A Sucan, Isaac I.Y. Saito, Michael Ferguson, Scott K Logan

0.8.8 (2014-10-01)
------------------
* Merge pull request `#13 <https://github.com/ros-planning/warehouse_ros/issues/13>`_ from corot/master
  Issue `#11 <https://github.com/ros-planning/warehouse_ros/issues/11>`_: Add a Python library
* Merge pull request `#15 <https://github.com/ros-planning/warehouse_ros/issues/15>`_ from v4hn/shared-static-mongodb
  only export MongoDB dependency for shared mongodb-library
* only export MongoDB dependency for shared mongodb-library
  libmongoclient.a uses quite a number of other libs and the exact
  requirements can't be read from a cmake/pc file.
  Therefore it makes more sense to keep the dependency hidden from ROS
  when we use the static lib. libwarehouse_ros then provides all required functions.
  ... This is a bit like creating a libmongoclient.so, but the whole problem
  exists because debian/ubuntu don't provide this one, right?
  The shared library can - and has to - be exported as a dependency to ROS.
* Missing part of https://github.com/corot/world_canvas/issues/10:
  requires both mongodb and mongodb-dev
* Merge branch 'master' of https://github.com/corot/warehouse_ros.git
* Add kwargs also to insert so we can solves issues as
  https://github.com/corot/world_canvas/issues/13
* Add kwargs to ensure_index so we can solves issues as
  https://github.com/corot/world_canvas/issues/13
* Add python-pymongo dependency
* Issue https://github.com/corot/world_canvas/issues/11: rospy queue_size
  warnings
* Issue `#11 <https://github.com/ros-planning/warehouse_ros/issues/11>`_: Add a Python library
* Contributors: Ioan A Sucan, corot, v4hn

0.8.5 (2014-02-23)
------------------
* Fixed malloc.h inclusion on Mac OS X
* Rename README.rst to README.md
* added travis support
* Contributors: Acorn, Dave Hershberger, Ioan A Sucan, Marco Esposito

0.8.4 (2013-07-03)
------------------
* update how we find MongoDB

0.8.2 (2013-07-03)
------------------
* fix typo and use correct install location
* add config.h.in for deciding how to include mongo headers

0.8.1 (2013-07-03)
------------------
* fix linking issues (missing SSL symbols) in deps, undef defined macros
