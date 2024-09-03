cmake_minimum_required(VERSION 2.6)
PROJECT(  speech )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior EmotionalMotor Recorder Speech )
