^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_grasping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2024-11-08)
------------------
* remove unported script
* add documentation
* forward port continuous detection (`#16 <https://github.com/mikeferguson/simple_grasping/issues/16>`_)
  forward port of `#7 <https://github.com/mikeferguson/simple_grasping/issues/7>`_
* add support for QoS overrides (`#15 <https://github.com/mikeferguson/simple_grasping/issues/15>`_)
* add continuous integration (`#14 <https://github.com/mikeferguson/simple_grasping/issues/14>`_)
  targeting iron only right now - grasping_msgs just released into jazzy
* cleanup dependencies and build, works on jazzy
* replace c-style cast
* fix issues in package.xml
  trying to get the build farm to succeed on source job
* add LICENSE file
* updates for ROS2 humble (`#9 <https://github.com/mikeferguson/simple_grasping/issues/9>`_)
* initial port to ros2 (`#6 <https://github.com/mikeferguson/simple_grasping/issues/6>`_)
* Contributors: Michael Ferguson

0.3.1 (2018-08-14)
------------------
* insert proper key name
* Contributors: Michael Ferguson

0.3.0 (2018-08-14)
------------------
* Merge pull request `#4 <https://github.com/mikeferguson/simple_grasping/issues/4>`_ from mikaelarguedas/patch-1
  add libvtk6-qt-dev dependency to build on Debian Stretch
* add libvtk6-qt-dev dependency to build on Debian Stretch
  the `libpcl-dev` package is missing a dependency causing the `Qt5::Widgets` target to be exported but not satisfied.
* Contributors: Michael Ferguson, Mikael Arguedas

0.2.2 (2015-04-24)
------------------
* parameterize gripper opening tolerance
* Contributors: Michael Ferguson

0.2.1 (2015-04-05)
------------------
* create standalone grasp_planner_node
* Contributors: Michael Ferguson

0.2.0 (2015-03-18)
------------------
* Initial release
* Contributors: Michael Ferguson
