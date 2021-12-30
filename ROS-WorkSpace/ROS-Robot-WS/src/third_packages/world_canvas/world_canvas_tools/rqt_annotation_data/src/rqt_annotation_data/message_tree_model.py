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

import threading

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QStandardItem

from rqt_py_common.message_tree_model import MessageTreeModel
from rqt_py_common.data_items import ReadonlyItem, CheckableItem


class MessageTreeModel(MessageTreeModel):
    _column_names = ['field', 'type', 'expression']
    item_value_changed = Signal(int, str, str, str, object)

    def __init__(self, parent=None):
        super(MessageTreeModel, self).__init__(parent)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)
        
        self.annotation = None

    def clear(self):
        super(MessageTreeModel, self).clear()
        self.setHorizontalHeaderLabels(self._column_names)

    def get_message_ids(self, index_list):
        return [item._user_data['message_id'] for item in self._get_toplevel_items(index_list)]

    def remove_items_with_parents(self, index_list):
        for item in self._get_toplevel_items(index_list):
            self.removeRow(item.row())

    def handle_item_changed(self, item):
        if not self._item_change_lock.acquire(False):
            #qDebug('MessageTreeModel.handle_item_changed(): could not acquire lock')
            return
        # lock has been acquired
        topic_name = item._path
        column_name = self._column_names[item.column()]
        if item.isCheckable():
            new_value = str(item.checkState() == Qt.Checked)
        else:
            new_value = item.text().strip()
        #print 'MessageTreeModel.handle_item_changed(): %s, %s, %s' % (topic_name, column_name, new_value)

        self.item_value_changed.emit(item._user_data['message_id'], topic_name, column_name, new_value, item.setText)

        # release lock
        self._item_change_lock.release()

    def remove_message(self, message_id):
        for top_level_row_number in range(self.rowCount()):
            item = self.item(top_level_row_number)
            if item is not None and item._user_data['message_id'] == message_id:
                self.removeRow(top_level_row_number)
                return top_level_row_number
        return None

    def update_message(self, message_info):
        top_level_row_number = self.remove_message(message_info['message_id'])
        self.add_message(message_info, top_level_row_number)

    def add_message(self, message_info, top_level_row_number=None, annotation=None):
        # recursively create widget items for the message's slots
        parent = self
        slot = message_info['instance']
        slot_name = message_info['annot_name']           
        slot_type = message_info['instance']._type
        slot_path = message_info['topic_name']
        user_data = {'message_id': message_info['message_id']}
        kwargs = {
            'user_data': user_data,
            'top_level_row_number': top_level_row_number,
            'expressions': message_info['expressions'],
        }
        self.annotation = annotation
        top_level_row = self._recursive_create_items(parent, slot, slot_name, slot_type, slot_path, **kwargs)

    def _get_data_items_for_path(self, slot_name, slot_type, slot_path, **kwargs):
        expression_item = QStandardItem('')
        expression_item.setToolTip('enter valid Python expression here, using "i" as counter and functions from math, random and time modules')
        return (ReadonlyItem(slot_name), ReadonlyItem(QStandardItem(slot_type)), expression_item)

    def _recursive_create_items(self, parent, slot, slot_name, slot_type, slot_path, expressions={}, **kwargs):
        row, is_leaf_node = super(MessageTreeModel, self)._recursive_create_items(parent, slot, slot_name, slot_type, slot_path, expressions=expressions, **kwargs)
        if is_leaf_node:
            expression_text = ''
            if self.annotation is not None:
                expression_text = str(self._get_annot_value(slot_name, slot_type, slot_path))
            if expression_text:
                # We found a possibly matching field in the annotation
                row[self._column_index['expression']].setText(expression_text)
                self.handle_item_changed(row[-1])
            else:
                row[self._column_index['expression']].setText(expressions.get(slot_path, repr(slot)))
        return row

    def _get_annot_value(self, slot_name, slot_type, slot_path):
        # Look for typical fields that can be matched to annotaion's ones
        if slot_type == 'string' and slot_path.endswith('name'):
            return self.annotation.name
        if slot_type.startswith('float') and slot_path.endswith('length'):
            return self.annotation.size.x
        if slot_type.startswith('float') and slot_path.endswith('width'):
            return self.annotation.size.y
        if slot_type.startswith('float') and slot_path.endswith('height'):
            return self.annotation.size.z
        if slot_type == 'string' and slot_path.endswith('pose/header/frame_id'):
            return self.annotation.pose.header.frame_id
        if slot_type == 'float64' and slot_path.endswith('pose/position/x'):
            return self.annotation.pose.pose.pose.position.x
        if slot_type == 'float64' and slot_path.endswith('pose/position/y'):
            return self.annotation.pose.pose.pose.position.y
        if slot_type == 'float64' and slot_path.endswith('pose/position/z'):
            return self.annotation.pose.pose.pose.position.z
        if slot_type == 'float64' and slot_path.endswith('pose/orientation/x'):
            return self.annotation.pose.pose.pose.orientation.x
        if slot_type == 'float64' and slot_path.endswith('pose/orientation/y'):
            return self.annotation.pose.pose.pose.orientation.y
        if slot_type == 'float64' and slot_path.endswith('pose/orientation/z'):
            return self.annotation.pose.pose.pose.orientation.z
        if slot_type == 'float64' and slot_path.endswith('pose/orientation/w'):
            return self.annotation.pose.pose.pose.orientation.w
        
        return ''