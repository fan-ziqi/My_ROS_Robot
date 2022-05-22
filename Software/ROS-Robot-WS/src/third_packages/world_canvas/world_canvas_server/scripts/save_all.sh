#!/bin/bash

rosrun world_canvas_server save_markers.py _world:=$1 _file:=`rospack find world_canvas_server`/test/annotations/ar_list.yaml
rosrun world_canvas_server save_tables.py  _world:=$1 _file:=`rospack find world_canvas_server`/test/annotations/table_list.yaml
rosrun world_canvas_server save_columns.py _world:=$1 _file:=`rospack find world_canvas_server`/test/annotations/column_list.yaml
rosrun world_canvas_server save_walls.py   _world:=$1 _file:=`rospack find world_canvas_server`/test/annotations/wall_list.yaml
