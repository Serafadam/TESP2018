"""
Created by Alejandro Daniel Noel
"""

import sounddevice as sd
import numpy as np
from time import sleep


sampling_freq = 48000


class Wave(object):
    def __init__(self, wave=None, channel=1):
        self.wave = wave if wave is not None else np.ones(sampling_freq)
        self._delay = 0.0
        self._shift = 0.0
        self._attack = 0.0
        self._sustain = 0.0
        self._release = 0.0
        self.channels = np.array([channel]) if channel is 1 else channel

    def __add__(self, other):
        max_chan = max(max(self.channels), max(other.channels))
        self.make_up_to_channel(max_chan)
        other.make_up_to_channel(max_chan)
        new_channels = np.array(list(set(list(np.concatenate((self.channels, other.channels))))))
        np.sort(new_channels)
        return Wave(self.wave + other.wave, channel=new_channels)

    def __mul__(self, other):
        if isinstance(other, Wave):
            max_chan = max(max(self.channels), max(other.channels))
            self.make_up_to_channel(max_chan)
            other.make_up_to_channel(max_chan)
            new_channels = np.array(list(set(list(np.concatenate((self.channels, other.channels))))))
            np.sort(new_channels)
            return Wave(self.wave * other.wave, channel=new_channels)
        else:
            return Wave(self.wave * other, channel=self.channels)

    @property
    def channel(self):
        return self.channels

    @channel.setter
    def channel(self, value):
        assert self.channels.size == 1
        self.channels = np.array([value])
        if value > 1:
            self.wave = np.column_stack((np.zeros((sampling_freq, value - 1)), self.wave))

    def make_up_to_channel(self, channel_num):
        if channel_num - max(self.channels) > 0:
            self.wave = np.column_stack((self.wave, np.zeros((sampling_freq, channel_num - max(self.channels)))))

    @property
    def delay(self):
        return self._delay

    @delay.setter
    def delay(self, value):
        self._delay = value
        pos = int(value * sampling_freq)
        self.wave = np.roll(self.wave, pos)
        self.wave[:pos] *= 0.0

    @property
    def shift(self):
        return self._shift

    @shift.setter
    def shift(self, value):
        self._delay = value
        pos = int(value * sampling_freq)
        self.wave = np.roll(self.wave, pos)

    @property
    def attack(self):
        return self._attack

    @attack.setter
    def attack(self, value):
        self._attack = value
        points = int(value * sampling_freq)
        start = int(self.delay * sampling_freq)
        self.wave[start:start + points] *= np.linspace(0.0, 1.0, points)

    @property
    def sustain(self):
        return self._sustain

    @sustain.setter
    def sustain(self, value):
        self._sustain = value

    @property
    def release(self):
        return self._release

    @release.setter
    def release(self, value):
        self._release=value
        points= int(value * sampling_freq)
        start= int((self.sustain + self.attack +self.delay) * sampling_freq)
        self.wave[start:start + points] *= np.linspace(1.0, 0.0, points)
        self.wave[start + points:] *= 0.0

    def play(self, duration, blocking=False, device=0):
        sd.play(self.wave, loop=True, mapping=self.channels, device=device)
        if blocking:
            sleep(duration)
            sd.stop()

    def stop(self):
        sd.stop()


class Sine(Wave):
    def __init__(self, frequency, amplitude=1.0, **kwargs):
        self.frequency = frequency
        self.amplitude = amplitude
        super(Sine,self).__init__(np.sin(np.linspace(0.0, 2 * np.pi * frequency, sampling_freq)), **kwargs)


if __name__ == "__main__":
    awave1 = Sine(frequency=142)
    awave1.delay = 0.231
    awave1.attack = 0.071
    awave1.sustain = 0.479
    awave1.release = 0.043
    awave1.channel = 1

    awave2 = Sine(frequency=125)
    awave2.delay = 0.1
    awave2.attack = 0.059
    awave2.sustain = 0.465
    awave2.release = 0.040
    awave2.channel = 2

    awave3 = Sine(frequency=125)
    awave3.delay = 0.3
    awave3.attack = 0.059
    awave3.sustain = 0.465
    awave3.release = 0.040
    awave3.channel = 3

    awave4 = Sine(frequency=125)
    awave4.delay = 0.0
    awave4.attack = 0.059
    awave4.sustain = 0.465
    awave4.release = 0.040
    awave4.channel = 4

    awave5 = awave1 + awave2 + awave3 + awave4
    awave5 *= 0.5
    awave5.play(1.0, blocking=True, device=2)
