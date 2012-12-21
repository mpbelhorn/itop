=============
MotionControl
=============

Motion control, DAQ, analysis code, etc for the UC Belle group optics lab.

Simple usage::

    #!/usr/bin/python

    from motioncontrol import controller as sc

    eps = sc.StageController('COM3')
    mirror = eps.axis1
    ccd_x = eps.axis2
    ccd_y = eps.axis3

    ccd_x.goToHome()

StageController
===============

This module covers communication and control of an EPS300 controller itself.
It contains pointers to each of:

* axis1, axis2, and axis3 corresponding to the numbered
  axes on the controller. Unused axes are still given pointers. Communication
  to unused axes results in a controller error.

* The serial buffer on the EPS controller.

Stage
=====

The Stage module contains commands to manipulate and group individual stages.

Most stepper-based (particularly CC series) stages are accomodated, as well
as all stage formats (linear, rotational, pan-tilt, etc).

The various categories of commands are outlined as follows.

I/0 Methods
-----------

Status Functions
----------------

Motion and Position Control
---------------------------

Motion Device Parameters
------------------------

On-board Programming
--------------------

Not yet implemented.

Trajectory Definition
---------------------

Flow Control and Sequencing
---------------------------

Group Functions
---------------

Not yet implemented.

Digital Filters
---------------

Not implemented.

Master-Slave Mode Definition
----------------------------

Not yet implemented.

Urls are http://like.this and links can be
written `like this <http://www.example.com/foo/bar>`_.
