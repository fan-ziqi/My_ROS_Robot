#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jorge Santos

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon, QWidget

import roslib
import rosmsg
import rospkg
import rospy

from qt_gui_py_common.worker_thread import WorkerThread
from rqt_py_common.extended_combo_box import ExtendedComboBox
from .message_tree_widget import MessageTreeWidget


# main class inherits from the ui window class
class MsgEditorWidget(QWidget):
    msg_type_changed = Signal(str)
    change_message = Signal(int, str, str, str, object)
    clean  = Signal()
    accept = Signal()
    cancel = Signal()

    def __init__(self, parent=None):
        super(MsgEditorWidget, self).__init__(parent)
        self._topic_dict = {}
        self._update_thread = WorkerThread(self._update_thread_run, self._update_finished)

        self._rospack = rospkg.RosPack()
        ui_file = os.path.join(self._rospack.get_path('rqt_annotation_data'), 'resource', 'MsgEditor.ui')
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox, 'MessageTreeWidget': MessageTreeWidget})

        self.refresh_combo_boxes()

        self.message_tree_widget.model().item_value_changed.connect(self.change_message)
        self.clear_button.clicked.connect(self.clean)

    def shutdown_plugin(self):
        self._update_thread.kill()

    @Slot()
    def refresh_combo_boxes(self):
        self._update_thread.kill()
        self.msg_type_combo_box.setEnabled(False)
        self.msg_type_combo_box.setEditText('updating...')
        self._update_thread.start()

    # this runs in a non-gui thread, so don't access widgets here directly
    def _update_thread_run(self):
        # update msg_type_combo_box
        message_type_names = []
        try:
            # this only works on fuerte and up
            packages = sorted([pkg_tuple[0] for pkg_tuple in rosmsg.iterate_packages(self._rospack, rosmsg.MODE_MSG)])
        except:
            # this works up to electric
            packages = sorted(rosmsg.list_packages())
        for package in packages:
            for base_type_str in rosmsg.list_msgs(package, rospack=self._rospack):
                message_class = roslib.message.get_message_class(base_type_str)
                if message_class is not None:
                    message_type_names.append(base_type_str)

        if hasattr(self, 'prev_type_name'):
            del self.prev_type_name
        message_type_names.append('') # null message type at first
        self.msg_type_combo_box.setItems.emit(sorted(message_type_names))

        # update topic_combo_box
        _, _, topic_types = rospy.get_master().getTopicTypes()
        self._topic_dict = dict(topic_types)

    @Slot()
    def _update_finished(self):
        self.msg_type_combo_box.setEnabled(True)

    @Slot(str)
    def on_msg_type_combo_box_currentIndexChanged(self, type_name):
        self.msg_type_changed.emit(type_name)
        self.prev_type_name = str(self.msg_type_combo_box.currentText())

    @Slot()
    def on_accept_button_clicked(self):
        self.accept.emit()

    def on_cancel_button_clicked(self):
        self.cancel.emit()