#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2024 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.

import sys, Ice, os
from PySide6 import QtWidgets, QtCore

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior




class GenericWorker(QtCore.QObject):

    """
    Initializes a worker object that can be controlled by signals and slots,
    enabling it to be killed remotely via a "kill" signal. It also allows setting
    a timer period in milliseconds, restarting it as needed.

    Attributes:
        kill (QtCoreSignal): Initialized with a value of `QtCore.Signal()`. It
            represents a signal emitted by the worker to indicate that it should
            be terminated.
        emotionalmotor_proxy (Dict[str,QObject]): Initialized from a dictionary
            `mprx`. It references a proxy object for Emotional Motor functionality,
            likely related to motor control or manipulation.
        recorder_proxy (Dict[str,any]|Dict[str,object]): Initialized with a value
            from the dictionary `mprx`. It is assigned the proxy to a recorder
            object. The specific details are not shown in this code snippet.
        mutex (QtCoreQMutex): Created to be used as a mutex (short for mutual
            exclusion), a synchronization primitive that allows only one thread
            to access shared resources at a time, providing data protection.
        Period (int): 30 by default, representing a timer period in milliseconds
            that controls when events are triggered.
        timer (QtCoreQTimer|None): Used to trigger a signal at regular intervals
            based on its Period value. It is set up to start when the Period is
            updated or initially configured with a Period of 30 seconds.

    """
    kill = QtCore.Signal()

    def __init__(self, mprx):
        """
        Initializes and sets up an instance with its parent object, assigning
        specified proxies to specific attributes, creating a mutex for thread
        safety, setting a timer interval, and instantiating a QTimer object
        associated with the GenericWorker instance.

        Args:
            mprx (Dict[str, object]): Expected to be a dictionary containing proxies
                for other objects. Specifically, it contains keys "EmotionalMotorProxy"
                and "RecorderProxy", each associated with an instance of a class
                implementing those interfaces.

        """
        super(GenericWorker, self).__init__()

        self.emotionalmotor_proxy = mprx["EmotionalMotorProxy"]
        self.recorder_proxy = mprx["RecorderProxy"]

        self.mutex = QtCore.QMutex()
        self.Period = 30
        self.timer = QtCore.QTimer(self)


    @QtCore.Slot()
    def killYourSelf(self):
        """
        Emits a signal to kill itself, logging a debug message "Killing myself"
        before doing so. This implies self-termination upon a certain condition
        or user request.

        """
        rDebug("Killing myself")
        self.kill.emit()

    # \brief Change compute period
    # @param per Period in ms
    @QtCore.Slot(int)
    def setPeriod(self, p):
        """
        Updates the period value, prints a message with the new period value, and
        starts a timer with the updated period.

        Args:
            p (int): Referenced as a period value. It is intended to represent an
                interval duration, possibly for timing-related purposes within the
                class's functionality.

        """
        print("Period changed", p)
        self.Period = p
        self.timer.start(self.Period)
