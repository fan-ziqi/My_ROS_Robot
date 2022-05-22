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
import sys
import math
import random
import time

from python_qt_binding.QtGui import QMessageBox
from python_qt_binding.QtCore import Slot, QSignalMapper, QTimer, qWarning

import roslib
import rospy
import genpy
from rqt_gui_py.plugin import Plugin
from rqt_py_common.topic_helpers import get_field_type
from rqt_annotation_data.msg_editor_widget import MsgEditorWidget

import std_msgs.msg
from world_canvas_msgs.msg import *
from world_canvas_msgs.srv import *
from world_canvas_utils.serialization import *

class MsgEditor(Plugin):

    def __init__(self, context):
        super(MsgEditor, self).__init__(context)
        self.setObjectName('MsgEditor')

        # create widget
        self._widget = MsgEditorWidget()
        self._widget.accept.connect(self.accept)
        self._widget.cancel.connect(self.cancel)
        self._widget.clean.connect(self.clean_up_message)
        self._widget.msg_type_changed.connect(self.msg_type_changed)
        self._widget.change_message.connect(self.change_message)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # create context for the expression eval statement
        self._eval_locals = {'i': 0}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        # add our self to the main window
        context.add_widget(self._widget)
        
        self.annotation = None
        self.edit_data_srv = \
            rospy.Service('/__edit_annotation_data__', EditAnnotationsData, self.on_edit_data_srv)

    def on_edit_data_srv(self, request):
        # Request from the annotations editor
        # TODO: somehow all should be blocked until we receive this request...
        
        self.annotation = request.annotation
        
        if request.action == EditAnnotationsDataRequest.EDIT:
            msg_class = roslib.message.get_message_class(request.data.type)
            if msg_class is None:
                # This could happen if the message type is wrong or not known for this node (i.e. its
                # package is not on ROS_PACKAGE_PATH). Both cases are really weird in the client side.
                message = "Data type '%s' definition not found" % d.type
                rospy.logerr(message)
                raise Exception(message)
            try:
                old_value = deserialize_msg(request.data.data, msg_class)
            except SerializationError as e:
                message = "Deserialization failed: %s" % str(e)
                rospy.logerr(message)
                raise Exception(message)
            
            self.clean_up_message()
            self.message_info = {
                'type_name': request.data.type,
                'instance': old_value,
            }
            self._set_message(self.message_info)

        # We let now the user to do whatever he wants, until he presses accept or cancel
        while not hasattr(self, 'user_action'):
            rospy.sleep(0.5)

        response = EditAnnotationsDataResponse()
        response.action = self.user_action

        if response.action == EditAnnotationsDataResponse.UPDATE:
            # User pressed accept and we have a message under edition;
            # serialize it and send back to the main annotations editor
            self._fill_message_slots(self.message_info['instance'], self.message_info['topic_name'],
                                     self.message_info['expressions'], self.message_info['counter'])
            response.data.id = request.data.id  # keep same uuid
            response.data.type = self.message_info['type_name']
    
            try:
                response.data.data = serialize_msg(self.message_info['instance'])
            except SerializationError as e:
                message = "Serialization failed: %s" % str(e)
                rospy.logerr(message)
                raise Exception(message)

        return response

    @Slot()
    def accept(self):
        if hasattr(self, 'message_info'):
            self.user_action = EditAnnotationsDataResponse.UPDATE
        else:
            # User pressed accept buy we have not message under edition;
            # if the user confirms, the current data will be discarded
            answer = QMessageBox.question(self._widget, 'Delete Existing Message', 
                    'No message under edition. Continue will delete any existing data\n'\
                    'Are you sure?', QMessageBox.Yes, QMessageBox.Cancel)
            if answer != QMessageBox.Yes:
                return
            self.user_action = EditAnnotationsDataResponse.DELETE
        
        # Both on cancel and accept we let some margin for the service handler to complete
        rospy.sleep(0.5)
        self.shutdown_plugin()
        sys.exit(0)

    @Slot()
    def cancel(self):
        self.user_action = EditAnnotationsDataResponse.CANCEL
        rospy.sleep(0.5)
        self.shutdown_plugin()
        sys.exit(0)

    @Slot(str, str, float, bool)
    def msg_type_changed(self, type_name):
        if not type_name:
            if hasattr(self, 'message_info'):
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
            return    # initialization with empty string; just ignore
        if self._create_message_instance(type_name) is None:
            QMessageBox.critical(self._widget, 'Change Message Type', 
                    'Unrecognized message type', QMessageBox.Ok)
            if hasattr(self, 'message_info'):
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
            else:
                self._widget.msg_type_combo_box.setEditText('')
            return
        if hasattr(self, 'message_info'):
            if self.message_info['type_name'] == type_name:
                return    # selected same type as current, just ignore

            answer = QMessageBox.question(self._widget, 'Change Message Type', 
                    'Are you sure you want to change current message type?\n'\
                    'All changes will be discarded!', QMessageBox.Ok, QMessageBox.Cancel)
            if answer != QMessageBox.Ok:
                self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
                return
        
            self.clean_up_message()

        self.message_info = {
            'type_name': str(type_name),
            'instance': self._create_message_instance(str(type_name))
        }
        
        # Ask for filling the message with annotation's fields likely to be the same
        answer = QMessageBox.question(self._widget, "Fill Message's Fields", 
                "Do you want to copy matching fields from the annotation?\n" \
                "You can change the suggested values later", QMessageBox.Yes, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self._set_message(self.message_info, self.annotation)
        else:
            self._set_message(self.message_info)

    def _set_message(self, message_info, annotation=None):
        message_info['annot_name'] = self.annotation.name if self.annotation is not None else '__ERROR__'
        message_info['topic_name'] = '/__TOPIC__'
        message_info['message_id'] = 0
        message_info['counter'] = 0
        message_info['expressions'] = message_info.get('expressions', {})

        if message_info['instance'] is None:
            raise Exception("Cannot create instance of type '%s'" % message_info['type_name'])
        message_info['publisher'] = \
            rospy.Publisher(message_info['topic_name'], type(message_info['instance']), queue_size=1)

        self._widget.msg_type_combo_box.setEditText(self.message_info['type_name'])
        self._widget.message_tree_widget.model().add_message(message_info, annotation=annotation)

    @Slot(int, str, str, str, object)
    def change_message(self, message_id, topic_name, column_name, new_value, setter_callback):
        handler = getattr(self, '_change_message_%s' % column_name, None)
        if handler is not None:
            new_text = handler(self.message_info, topic_name, new_value)
            if new_text is not None:
                setter_callback(new_text)

    def _change_message_expression(self, message_info, topic_name, new_value):
        expression = str(new_value)
        if len(expression) == 0:
            if topic_name in message_info['expressions']:
                del message_info['expressions'][topic_name]
                #qDebug('MsgEditor._change_message_expression(): removed expression for: %s' % (topic_name))
        else:
            slot_type, is_array = get_field_type(topic_name)
            if is_array:
                slot_type = list
            # strip possible trailing error message from expression
            error_prefix = '# error'
            error_prefix_pos = expression.find(error_prefix)
            if error_prefix_pos >= 0:
                expression = expression[:error_prefix_pos]
            success, _ = self._evaluate_expression(expression, slot_type)
            if success:
                old_expression = message_info['expressions'].get(topic_name, None)
                message_info['expressions'][topic_name] = expression
                #print 'MsgEditor._change_message_expression(): topic: %s, type: %s, expression: %s' % (topic_name, slot_type, new_value)
                self._fill_message_slots(message_info['instance'], message_info['topic_name'], message_info['expressions'], message_info['counter'])
                try:
                    message_info['instance']._check_types()
                except Exception, e:
                    error_str = str(e)
                    print 'serialization error:', error_str
                    if old_expression is not None:
                        message_info['expressions'][topic_name] = old_expression
                    else:
                        del message_info['expressions'][topic_name]
                    return '%s %s: %s' % (expression, error_prefix, error_str)
                return expression
            else:
                return '%s %s evaluating as "%s"' % (expression, error_prefix, slot_type.__name__)

    def _extract_array_info(self, type_str):
        array_size = None
        if '[' in type_str and type_str[-1] == ']':
            type_str, array_size_str = type_str.split('[', 1)
            array_size_str = array_size_str[:-1]
            if len(array_size_str) > 0:
                array_size = int(array_size_str)
            else:
                array_size = 0

        return type_str, array_size

    def _create_message_instance(self, type_str):
        base_type_str, array_size = self._extract_array_info(type_str)

        try:
            base_message_type = roslib.message.get_message_class(base_type_str)
        except ValueError:
            base_message_type = None
        if base_message_type is None:
            print 'Could not create message of type "%s".' % base_type_str
            return None

        if array_size is not None:
            message = []
            for _ in range(array_size):
                message.append(base_message_type())
        else:
            message = base_message_type()
        return message

    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            successful_eval = False

        if slot_type is str:
            if successful_eval:
                value = str(value)
            else:
                # for string slots just convert the expression to str, if it did not evaluate successfully
                value = str(expression)
            successful_eval = True

        elif successful_eval:
            type_set = set((slot_type, type(value)))
            # check if value's type and slot_type belong to the same type group, i.e. array types, numeric types
            # and if they do, make sure values's type is converted to the exact slot_type
            if type_set <= set((list, tuple)) or type_set <= set((int, float)):
                # convert to the right type
                value = slot_type(value)

        if successful_eval and isinstance(value, slot_type):
            return True, value
        else:
            qWarning('MsgEditor._evaluate_expression(): failed to evaluate expression: "%s" as Python type "%s"' % (expression, slot_type.__name__))

        return False, None

    def _fill_message_slots(self, message, topic_name, expressions, counter):
        if topic_name in expressions and len(expressions[topic_name]) > 0:

            # get type
            if hasattr(message, '_type'):
                message_type = message._type
            else:
                message_type = type(message)

            self._eval_locals['i'] = counter
            success, value = self._evaluate_expression(expressions[topic_name], message_type)
            if not success:
                value = message_type()
            return value

        # if no expression exists for this topic_name, continue with it's child slots
        elif hasattr(message, '__slots__'):
            for slot_name in message.__slots__:
                value = self._fill_message_slots(getattr(message, slot_name), topic_name + '/' + slot_name, expressions, counter)
                if value is not None:
                    setattr(message, slot_name, value)

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            for index, slot in enumerate(message):
                self._fill_message_slots(slot, topic_name + '[%d]' % index, expressions, counter)

        return None

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def clean_up_message(self):
        if hasattr(self, 'message_info'):
            self._widget.msg_type_combo_box.setEditText('')
            self._widget.message_tree_widget.model().clear()
#             try:
#                 self.message_info['publisher'].unregister()
#             except KeyError:
#                 pass
            del self.message_info

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
        self.clean_up_message()
