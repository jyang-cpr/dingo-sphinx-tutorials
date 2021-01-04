Custom Computer Installation
==================================

Some Dingo-O and Dingo-D robots may be sold without a main PC installed.  If you have such a robot, you must provide
your own PC for the robot.

This section explains the process of physically connecting your computer to Dingo.  For OS and software installation,
see :doc:`Software Setup <software_setup>`.

.. note::

  For installing the OS on an Nvidia Jetson device, see :doc:<Jetson Xavier AGX `jetson_xavier`> or
  :doc:<Jetson Nano `jetson_nano`>.


Supported PCs
--------------

Dingo has mounting hole patterns for standard mini-ITX motherboards and Nvidia Jetson Nano, NX, & Xavier AGX computers.

.. note::

  Support for the Raspberry Pi 4 is coming at a future date.  For now there are no official mounting solutions, but
  you may be able to create your own bracket for connecting the Rasperry Pi family of computers to Dingo.

We recommend the following minimum requirements for any custom PC installed in Dingo:

.. note::

  Note that some accessories, such as 3d lidar, depth-sensing cameras, high-res cameras, and arms may have their
  own additional requirements that exceed these minimums.

* CPU: Quad-core amd64 or arm64 at 1.4GHz or faster
* RAM: 4GB
* Storage: 64GB
* At least 1 physical ethernet port


Powering the PC
----------------

.. warning::

  TODO


Installing a network switch
----------------------------

Because many sensors use ethernet connectivity you may find it useful to install a small ethernet switch inside Dingo's
main center bay.

.. warning::

  TODO
