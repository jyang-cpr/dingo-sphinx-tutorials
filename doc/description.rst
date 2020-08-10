dingo_description Package
===========================

The jackal_description package is the URDF robot description for Dingo

.. _Source: https://github.com/dingo-cpr/dingo


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Dingo.  For an example launchfile to use in visualizing this model, see `dingo_viz <http://wiki.ros.org/dingo_viz>`_.

.. image:: images/dingo_urdf.png
  :alt: Dingo model


Environment Variables
-----------------------

Dingo can be customized & extended through the use of several enviroment variables. These variables are listed below, along with a
summary of their effects and default values

.. raw:: html

    <table><tbody><tr>  <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_OMNI</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to 1 to switch from Dingo-D to Dingo-O (with omni-directional mecanum wheels)</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_URDF_EXTRAS</tt> </p></td>
      <td><p><tt>emptyu.urdf</tt> </p></td>
      <td><p>Specifies the path to a URDF file with which to extend the robot's physical configuraiton</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_CONTROL_EXTRAS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to 1 to enable loading <tt>DINGO_CONTROL_EXTRAS_PATH</tt></p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_CONTROL_EXTRAS_PATH</tt> </p></td>
      <td><p><i>undefined</i> </p></td>
      <td><p>Path to a YAML file on-disk that can be used to override or extend Dingo's default controls</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_CONFIG</tt> </p></td>
      <td><p><tt>base</tt> </p></td>
      <td><p>Specifies what pre-made sensor/mission configuration to load (see below)</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to 1 to equip Dingo with a lidar unit</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER_HOKUYO</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Use a Hokuyo UST-10 lidar instead of the default SICK LMS-1xx. Ignored if <tt>DINGO_LASER</tt> is <tt>0</tt>.</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER_MOUNT</tt> </p></td>
      <td><p><tt>front</tt> </p></td>
      <td><p>Defines the mount point the Dingo's laser is connected to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER_TOPIC</tt> </p></td>
      <td><p><tt>front/scan</tt> </p></td>
      <td><p>The ROS topic that Dingo's lidar publishes on</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER_OFFSET</tt> </p></td>
      <td><p><tt>0.11 0 0</tt> </p></td>
      <td><p>XYZ offset for Dingo's lidar</p></td>
    </tr
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>DINGO_LASER_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>RPY offset for Dingo's lidar</p></td>
    </tr>
    </tbody></table>

Configurations
-----------------

As an alternative to individually specifying each accessory, some fixed configurations are provided in the package. These can be specified using the ``config arg to description.launch``, and are intended especially as a convenience for simulation launch.

====================================  ====================================================
Config:                               Description:
====================================  ====================================================
base                                  Base Dingo
front_laser                           Adds a SICK LMS1xx lidar to the Dingo's front mount
====================================  ====================================================

.. Note::

  Additional configurations coming soon
