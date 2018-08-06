#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import sys

from numpy import *


from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QApplication, QWidget
from python_qt_binding.QtWidgets import QVBoxLayout, QHBoxLayout
from python_qt_binding.QtWidgets import QLabel, QLineEdit, QPushButton, QSpinBox
from python_qt_binding import QtCore

import seaborn as sns
import matplotlib.pyplot as plt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.cm as cm
from vib_touch_msgs.msg import Spectrum

sys.setrecursionlimit(3000)

class SpectrogramPlugin(Plugin):
    update_signal = QtCore.pyqtSignal()
    subscriber_signal = QtCore.pyqtSignal(str)
    
    module_num_signal = QtCore.pyqtSignal()
    
    value_signal = QtCore.pyqtSignal(int)

    vib_freq = 48000
    overlap = 50
    stft_freq = 200
    
    frame_time = 1
    frame_length = frame_time * stft_freq

    label_flag = 0
    
    lowcut_freq = 0
    highcut_freq = vib_freq/2
    
    v_max = 0
    v_min = 0
    
    colorbarflag = 0
    
    cnt_fig = 0

    def __init__(self, context):
        super(SpectrogramPlugin, self).__init__(context)
        self.setObjectName('Spectrogram')
        
        sns.set(style="whitegrid", palette="bright", color_codes=True)

        self._widget = QWidget()
        layout = QVBoxLayout()
        self._widget.setLayout(layout)
        
        layout_ = QHBoxLayout()
        self.lbl_topic = QLabel('Topic:')
        layout_.addWidget(self.lbl_topic)
        self.le_topic = QLineEdit()
        layout_.addWidget(self.le_topic)
        self.apply_topic = QPushButton("Apply")
        self.apply_topic.clicked.connect(self.apply_clicked_topic)
        layout_.addWidget(self.apply_topic)
        layout.addLayout(layout_)
        
        layout_ = QHBoxLayout()
        self.lbl_lcf = QLabel('Low-cut Freq.[Hz]:')
        layout_.addWidget(self.lbl_lcf)
        self.spb_lcf = QSpinBox()
        self.spb_lcf.setRange(0, 50)
        self.spb_lcf.setValue(0)
        layout_.addWidget(self.spb_lcf)
        self.apply_lcf = QPushButton("Apply")
        self.apply_lcf.clicked.connect(self.apply_clicked_lcf)
        layout_.addWidget(self.apply_lcf)
        layout.addLayout(layout_)
        
        layout_ = QHBoxLayout()
        self.lbl_hcf = QLabel('High-cut Freq.[Hz]:')
        layout_.addWidget(self.lbl_hcf)
        self.spb_hcf = QSpinBox()
        self.spb_hcf.setRange(50, self.vib_freq/2)
        self.spb_hcf.setValue(self.vib_freq/2)
        layout_.addWidget(self.spb_hcf)
        self.apply_hcf = QPushButton("Apply")
        self.apply_hcf.clicked.connect(self.apply_clicked_hcf)
        layout_.addWidget(self.apply_hcf)
        layout.addLayout(layout_)


        #self.fig, self.axes = plt.subplots(2, 1, sharex=True)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)
        self.canvas = FigureCanvas(self.fig)
        self.fig.tight_layout()
        layout.addWidget(self.canvas)

        context.add_widget(self._widget)

        self.update_signal.connect(self.update_spectrogram)
        self.subscriber_signal.connect(self.update_subscriber)
        self.subscriber_signal.emit('spectrum')

    def changed_spinbox_value(self, n):
        self.valueChanged.emit(n)

    def spectrum_callback(self, data):
        nch = data.nch
        len = data.nfreq

        if self.spectrogram is None:
            self.spectrogram = zeros([len, self.frame_length, nch])
            #self.spectrogram = ones([len, self.frame_length, 4*nch])*log(1e-8)
            
        self.spectrogram = roll(self.spectrogram, -1, 1)
        
        for i in range(nch): 
            s = array(data.data).reshape([nch, len, 2])[i]
            s = linalg.norm(s, axis=1)
            s += 1e-8
            log(s, s)
            self.spectrogram[:, -1, i] = s
            
        #if data.header.seq % 2 == 0:
        if self.v_max < self.spectrogram[self.lowcut_freq:self.highcut_freq,-1,i].max():
            self.v_max = self.spectrogram[self.lowcut_freq:self.highcut_freq,-1,i].max()
        if self.v_min > self.spectrogram[self.lowcut_freq:self.highcut_freq,-1,i].min():
            self.v_min = self.spectrogram[self.lowcut_freq:self.highcut_freq,-1,i].min()
        self.update_signal.emit()

    def apply_clicked_topic(self):
        self.update_subscriber(self.le_topic.displayText())
    
    def apply_clicked_lcf(self):
        self.lowcut_freq = self.spb_lcf.value()
    
    def apply_clicked_hcf(self):
        self.highcut_freq = self.spb_hcf.value()

    def update_spectrogram(self):
        if self.spectrogram is not None:
            
            yticks = [0, self.highcut_freq/2/self.stft_freq-self.lowcut_freq*2/self.stft_freq, self.highcut_freq/self.stft_freq-self.lowcut_freq*2/self.stft_freq, self.highcut_freq/2*3/self.stft_freq-self.lowcut_freq*2/self.stft_freq, self.highcut_freq*2/self.stft_freq-self.lowcut_freq*2/self.stft_freq]
            yticks_label = [self.lowcut_freq, self.highcut_freq/4, self.highcut_freq/2, self.highcut_freq/4*3, self.highcut_freq]
            
            xticks = [0, self.frame_length/4-1 , self.frame_length/2-1, self.frame_length*3/4-1, self.frame_length-1]
            xticks_label = [-self.frame_time*2, -self.frame_time/2*3 , -self.frame_time, -self.frame_time/2, 0]
            
            font_size = 13

            if self.cnt_fig%40 == 0:

                self.ax.clear()
                #self.im = self.ax.imshow(self.spectrogram[int(self.lowcut_freq*100/self.overlap/self.stft_freq):int(self.highcut_freq*100/self.overlap/self.stft_freq)+1,:,0], aspect="auto", origin="lower", cmap="winter", interpolation='none', vmin=self.v_min, vmax=self.v_max)
                self.im = self.ax.imshow(self.spectrogram[int(self.lowcut_freq*100/self.overlap/self.stft_freq):int(self.highcut_freq*100/self.overlap/self.stft_freq)+1,:,0], aspect="auto", origin="lower", cmap="jet", interpolation='none', vmin=12, vmax=16)
                self.ax.grid(None)
                self.ax.set_ylabel("Freq. [Hz]", fontsize = font_size, fontname='serif')
                self.ax.set_yticks(yticks)
                self.ax.set_yticklabels(["$%.1f$" % y for y in yticks_label], fontsize = font_size)
                self.ax.set_xticks(xticks)
                self.ax.set_xticklabels(["$%.1f$" % x for x in xticks_label], fontsize = font_size)
                self.ax.set_xlabel("Time [s]", fontsize = font_size, fontname='serif')
                
                if self.colorbarflag == 0:
                    self.colorbarflag = 1
                    self.cb = self.fig.colorbar(self.im)
                elif self.colorbarflag == 1:
                    self.cb.update_bruteforce(self.im)

                self.canvas.draw()

            self.cnt_fig += 1

        QApplication.processEvents()

    def update_subscriber(self, topic_name):
        self.topic_name = topic_name
        self.le_topic.setText(self.topic_name)

        if hasattr(self, 'sub'):
            self.sub.unregister()
        self.spectrogram = None
        #self.topic_name = 'spectrum'
        self.sub = rospy.Subscriber(topic_name,
                                    Spectrum, self.spectrum_callback,
                                    queue_size=5)

    def shutdown_plugin(self):
        pass
