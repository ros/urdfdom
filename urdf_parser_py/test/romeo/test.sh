#!/bin/bash
name=romeo
orig="$(rospack find romeo_description)/urdf/$name.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

dir=$(dirname $BASH_SOURCE)
$dir/../../scripts/display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" > "$patch"
