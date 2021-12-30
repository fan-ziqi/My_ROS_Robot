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

from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QAction, QIcon

from .message_tree_model import MessageTreeModel
from rqt_py_common.message_tree_widget import MessageTreeWidget
from rqt_py_common.item_delegates import SpinBoxDelegate


class MessageTreeWidget(MessageTreeWidget):
    remove_message = Signal(int)
    publish_once = Signal(int)

    def __init__(self, parent=None):
        super(MessageTreeWidget, self).__init__(parent)
        self.setModel(MessageTreeModel(self))
#        self._action_remove_message = QAction(QIcon.fromTheme('remove'), 'Remove Selected', self)
#        self._action_remove_message.triggered[bool].connect(self._handle_action_remove_message)
#        self._action_publish_once = QAction(QIcon.fromTheme('media-playback-start'), 'Publish Selected Once', self)
#        self._action_publish_once.triggered[bool].connect(self._handle_action_publish_once)
#        self.setItemDelegateForColumn(self.model()._column_index['rate'], SpinBoxDelegate(min_value=0, max_value=1000000, decimals=2))

    @Slot()
    def remove_selected_messages(self):
        message_ids = self.model().get_message_ids(self.selectedIndexes())
        for message_id in message_ids:
            self.remove_message.emit(message_id)
        self.model().remove_items_with_parents(self.selectedIndexes())

    def _context_menu_add_actions(self, menu, pos):
#         if self.selectionModel().hasSelection():
#             menu.addAction(self._action_remove_message)
#             menu.addAction(self._action_publish_once)
        # let super class add actions
        super(MessageTreeWidget, self)._context_menu_add_actions(menu, pos)

#     def _handle_action_remove_message(self, checked):
#         self.remove_selected_messages()
# 
#     def _handle_action_publish_once(self, checked):
#         for message_id in self.model().get_message_ids(self.selectedIndexes()):
#             self.publish_once.emit(message_id)
