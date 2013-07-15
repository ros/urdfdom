#!/bin/bash
name=romeo
orig="$(rospack find romeo_description)/urdf/$name.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

rosrun urdf_parser_py display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" > "$patch"