BelleII iTOP Quartz Testing
===========================

# Introduction

The BelleII imaging TOP detector is a barrel Cherenkov counter that uses
spherical mirrors to reduce ring smearing due to radiator thickness. The
detector also uses a set of expansion prisms to interface the Cherenkov
radiators to the PMT readout electronics. The physical properties of these
optics, mirrors and prisms, are verified to be within acceptable tolerances
at the University of Cincinnati. This software library drives the
instrumentation and analysis for the quality control testing of these optics.

# Components

The iTOP library consists of the following python modules.

## motioncontrol
The motioncontrol module provides robust control of Newport ESP driven
robotic stages via serial RS-232C link. The module provides representations
of ESP series stage controllers as well as an interface to ESP series
stages.

## math
The math module implements calculations used in the other iTOP library
modules. Submodules are provided for linear algebra calculations,
optics, and measurements with unertainties.

## beam
The beam module implements objects representing beam and mirror alignments,
beam trajectory segments, data points taken on the mirror, the beam imaging
CCD profiler, the beam trajectory tracking instrument (prfiler + xyzr stages),
and the mirror testing instrument as a whole (tracker + mirror).

## photodiode
The photodiode module provides an interface to the Hamamatsu C9329 photodiode
amplifers used to monitor beam power.

## raytrace
The raytrace module provides an environment for simulating the path of a beam
through a series of arbitrary optics. The module is used to simulate the
response of iTOP mirrors to the testing procedure.

## utilities
The utilities module provides generally useful utility functions shared by
other iTOP modules.

## analysis
The analysis module provides matplotlib based functions for analyzing recorded
data to determine the physical properties of the test optics.

