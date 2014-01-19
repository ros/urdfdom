#!/bin/bash

dir=$(dirname $BASH_SOURCE)
scripts="$dir/../../scripts"

name=calvin
orig="$dir/$name.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

$scripts/display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" > "$patch"
