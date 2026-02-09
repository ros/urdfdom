^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdfdom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.1.0 (2026-02-06)
------------------
* Support for URDF Specification 1.1
    * Add support for capsule geometry type (`#238 <https://github.com/ros/urdfdom/issues/238>`_)
      * Add documentation about versioning
      * Require version 2.1.0 of urdfdom_headers
      Co-authored-by: Steve Peters <scpeters@openrobotics.org>
    * Support quaternions in URDF 1.1 (`#235 <https://github.com/ros/urdfdom/issues/235>`_)
      Co-authored-by: Guillaume Doisy <doisyg@users.noreply.github.com>
* Fix multiple format-string vulnerabilities in URDF parser logging (`#243 <https://github.com/ros/urdfdom/issues/243>`_)
  User-controlled URDF content was passed directly to CONSOLE_BRIDGE_logError()
  at multiple call sites, allowing printf-style format string interpretation.
  All affected logging paths now use explicit "%s" format specifiers to ensure
  input is treated as data and to prevent information disclosure or undefined
  behavior.
* More logging format string fixes (`#244 <https://github.com/ros/urdfdom/issues/244>`_)
  * Add explicit "%s" format strings when logging
  * Use %s format string instead of string addition
* Read cmake version from package.xml (`#236 <https://github.com/ros/urdfdom/issues/236>`_)
  * Use regex to match version string.
  Based on suggestion from Chris Lalancette.
  * Require cmake minimum version 3.10
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: Florencia, Sai Kishor Kothakota, Steve Peters

5.0.4 (2025-12-08)
------------------
* Revert "Quaternion in urdf (PR123 new attempt) (#231)" (`#231 <https://github.com/ros/urdfdom/issues/231>`_)
* Contributors: Steve Peters

5.0.3 (2025-11-28)
------------------
* Quaternion in urdf (PR123 new attempt) (`#194 <https://github.com/ros/urdfdom/issues/194>`_)
* Removed tinyxml2_vendor dependency (`#225 <https://github.com/ros/urdfdom/issues/225>`_)
* Contributors: Alejandro Hernández Cordero, Guillaume Doisy

5.0.2 (2025-07-03)
------------------
* Relax the version compatibility for urdfdom_headers. (`#222 <https://github.com/ros/urdfdom/issues/222>`_)
* Contributors: Chris Lalancette

5.0.1 (2025-07-01)
------------------

4.0.3 (2025-06-23)
------------------
* Removed deprecated code (`#217 <https://github.com/ros/urdfdom/issues/217>`_)
* Remove ROS 1 workflows and update ROS 2 (`#218 <https://github.com/ros/urdfdom/issues/218>`_)
* Improvements for the URDF xsd specification  (`#200 <https://github.com/ros/urdfdom/issues/200>`_)
* Update ros2.yaml (`#214 <https://github.com/ros/urdfdom/issues/214>`_)
* fix: missing header (`#216 <https://github.com/ros/urdfdom/issues/216>`_)
* Contributors: Alejandro Hernández Cordero, Amin Ya, Pierre Ballif, mosfet80
