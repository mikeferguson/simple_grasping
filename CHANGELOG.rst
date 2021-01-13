^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_grasping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2021-01-13)
------------------
* add depend on grasping_msgs, fix `#5 <https://github.com/mikeferguson/simple_grasping/issues/5>`_
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
