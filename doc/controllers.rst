Dingo Controller Pairing
===========================

Dingo ships with a PS4 controller by default that should already be paired with your robot.

If the controller does not pair automatically, or if you want to pair a new controller with your computer
to control Dingo in simulation, follow these steps:

1. Install ds4drv
-------------------

Install the ``python-ds4drv`` package on Ubuntu:

.. code-block:: bash

  sudo apt-get install python-ds4drv

2. Put the controller in pairing mode
---------------------------------------

Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white

3. Run ds4drv-pair
--------------------

From the terminal, run

.. code-block:: bash

  sudo ds4drv-pair

This script will scan for nearby Bluetooth devices, and connect automatically.

4. Check udev rules
---------------------

On Dingo, once the controller is paired you should see the file ``/dev/input/ps4`` appear on your system:

.. code-block:: bash

  ls -l /dev/input
  [...]
  crw-rw-rw-+ 1 root root  13,  0 Aug 10 12:16 js0
  [...]
  lrwxrwxrwx  1 root root       3 Aug 10 12:16 ps4 -> js0
  [...]

If you do not see the ps4 device, add the following to ``/etc/udev/rules.d/41-playstation.rules``

.. code-block:: text

  KERNEL=="js*", SUBSYSTEM=="input", ATTRS{name}=="Wireless Controller", MODE="0666", SYMLINK+="input/ps4"

and then reload udev:

.. code-block:: bash

  sudo udevadm control --reload-rules
  sudo udevadm trigger

5. Test the controller
-----------------------

You should now be able to run the following commands to make sure the controller input is working correctly:

.. code-block:: bash

  sudo apt-get install jstest-gtk
  jstest /dev/input/ps4
  [...]
  Axes:  0: -7770  1:-19256  2:-32767  3:     0  4:     0  5:-32767  6:-32767  7:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off

As you press buttons and move the joysticks you should see the axes and buttons update accordingly.

Finally, open two terminals.  In the first one, run:

.. code-block:: bash

  cd ~/catkin_ws
  source devel/setup.bash
  roslaunch dingo_control teleop.launch

And in the second one, run:

.. code-block:: bash

  cd ~/catkin_ws
  source devel/setup.bash
  rostopic echo /bluetooth_teleop/joy

As with ``jstest``, you should see the buttons and axes update as you use the controller.
