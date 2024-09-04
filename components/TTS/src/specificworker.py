#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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
#

import subprocess
import sys
from google_speech import Speech
# import pyttsx3
sys.path.append("../")
try:
	from Queue import Queue
except ImportError:
	from queue import Queue

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from genericworker import *

max_queue = 100
charsToAvoid = ["'", '"', '{', '}', '[', '<', '>', '(', ')', '&', '$', '|', '#']

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    """
    Implements a worker that reads text from a queue, processes it using various
    speech synthesis engines (Festival or Google TTS), and outputs synthesized
    speech to an emotional motor proxy. It also handles startup checks, timer
    events, and parameter updates.

    Attributes:
        Period (int): 2000 by default. It is used to specify the interval, in
            milliseconds, at which the worker's `compute` method will be executed
            when the `timer` starts.
        audioenviado (bool): Initialized to False in the `__init__` method. It
            does not seem to be used anywhere else in the code, implying it might
            be a leftover or unused variable.
        pitch (float|int): Initialized to a value of 1. It controls the pitch
            adjustment applied to speech synthesis, with its value being multiplied
            by 4 when set via the `Speech_setPitch` method.
        tempo (float): 1 by default, representing the playback speed of speech
            generated using `pyttshtts`. It can be modified by calling the
            `Speech_setTempo` method.
        text_queue (Queue[maxsize]): Initialized with a maximum size equal to
            max_queue, which is not defined within this code snippet. It stores
            text items to be processed by the `compute` method.
        startup_check (bool): Used as a parameter for its constructor (`__init__`).
            It determines whether the startup check method is called or not when
            creating an instance of this class.
        timer (QTimer|None): Initialized with a default value of None, which is
            then set to a QTimer object when the `startup_check` flag is False in
            the `__init__` method.
        compute (QtCorepyqtSignal|None): Associated with the slot method of the
            same name, which is connected to a QTimer's timeout signal. It is
            called when the timer times out.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes its attributes and sets up event handling for a timer, which
        triggers the compute function at regular intervals when startup checking
        is not enabled. Startup checking is an optional feature that can be disabled
        or enabled during initialization.

        Args:
            proxy_map (Dict[str, int]): Passed to the superclass's `__init__`
                method. The key-value pairs of this map are used for initializing
                the object with proxy information.
            startup_check (bool): Set to False by default. It determines whether
                or not to perform a startup check, which can be implemented in the
                `self.startup_check()` method if True. Otherwise, it enables timer
                functionality.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.audioenviado = False
        self.pitch = 1
        self.tempo = 1
        self.text_queue = Queue(max_queue)
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        """
        Sets or updates the value of the `_tts` attribute based on the presence
        of a "tts" key in the params dictionary. If present, it assigns the tts
        parameter to _tts; otherwise, it defaults to "festival". The function
        returns True.

        Args:
            params (Dict[str, str | bool | int]): Used to set various parameters
                for an object. It contains key-value pairs where keys are strings
                and values can be strings, boolean values, or integers.

        Returns:
            bool: True, regardless of whether the input parameters are valid or
            not. The code ignores any potential exceptions and continues executing.

        """
        if "tts" in params:
            self._tts = params["tts"]
        else:
            self._tts = "festival"
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True


    @QtCore.Slot()
    def compute(self):
        """
        Retrieves text from a queue, replaces specific characters with their escaped
        versions, and then passes it to another method called `habla`. This process
        is executed when there are items available in the queue.

        """
        if self.text_queue.empty():
            pass
        else:
            text_to_say = self.text_queue.get()
            for rep in charsToAvoid:
                text_to_say = text_to_say.replace(rep, '\\' + rep)
            #				shellcommand = "echo " + text_to_say  + " | padsp festival --tts"
            self.habla(text_to_say)

    def habla(self, text):
        """
        Initiates voice synthesis by printing a given text, engaging an emotional
        motor proxy, generating speech using pyttsx, and then disengaging the
        emotional motor proxy.

        Args:
            text (str | None): Used as input for printing, Google TTS synthesis
                via pyttshtts, and emotional motor interaction. Its presence is
                checked by default due to Python's syntax rules requiring a value
                in each argument position.

        """
        try:
            print(text)
            self.emotionalmotor_proxy.talking(True)            
            # self.recorder_proxy.recording(False)
            self.pyttshtts(text)
            # self.recorder_proxy.recording(True)
            self.emotionalmotor_proxy.talking(False)
        except ImportError:
            print("Problema say with googgle")
            print("\033[91m To use google TTS you need to install gTTS package and playsound\033[00m")
            print("\033[91m You can try to install it with pip install gTTS playsound\033[00m")

    def Speech_say(self, text, owerwrite):
        """
        Adds text to a queue for speech processing. If overwrite is True, it clears
        any existing items from the queue before adding the new text. The method
        returns True upon successful execution.

        Args:
            text (str | bytes): A message to be queued for speech output. It
                represents the actual text to be spoken, which can be either a
                string or bytes object.
            owerwrite (bool): Used to determine whether to reset the text queue
                or not when set to True, indicating that it should be overwritten
                with new text on each invocation.

        Returns:
            bool: `True`. This indicates that the text has been successfully added
            to the queue. The function always returns `True` regardless of whether
            the overwrite flag is set or not.

        """
        if owerwrite:
            self.text_queue = Queue(max_queue)
        self.text_queue.put(text)
        return True

    def pyttshtts(self, text):
        """
        Plays a synthesized text-to-speech output in Spanish, with custom pitch
        and tempo settings determined by the instance's pitch and tempo attributes.
        It utilizes the Speech object to generate speech from the input text.

        Args:
            text (str | bytes): Required, but its exact description cannot be
                determined from this snippet alone.

        """
        lang = "es"
        speech = Speech(text, lang)
        # speech.play()

        # you can also apply audio effects while playing (using SoX)
        # see http://sox.sourceforge.net/sox.html#EFFECTS for full effect documentation
        effects = ("pitch", str(self.pitch), "tempo", str(self.tempo))
        # sox_effects2 = ("tempo", "0.5")
        speech.play(effects)
        # speech.play(sox_effects2)

    def Speech_setPitch(self, pitch):

        """
        Sets and prints the pitch value to a local attribute, increasing it by a
        factor of four times the input value. This modification effectively amplifies
        the pitch before storing and displaying it. The method does not directly
        affect external speech properties.

        Args:
            pitch (float | int): Used to modify or set the pitch property, which
                controls the frequency or tone of the speech. Its value is multiplied
                by 4 and stored in an instance variable.

        """
        self.pitch = pitch*4
        print(self.pitch)

    def Speech_setTempo(self, tempo):

        """
        Sets a tempo value for speech, taking an input tempo and dividing it by
        10 to adjust it to an internal representation, then prints the adjusted value.

        Args:
            tempo (float | int): Intended to represent a tempo value, with a scaling
                factor implicitly applied within the function body. It appears to
                be normalized by a division by 10.

        """
        self.tempo = tempo/10
        print(self.tempo)

