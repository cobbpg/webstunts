#!/bin/sh

VERTEX_SHADERS=stunts
FRAGMENT_SHADERS=stunts

cp prelude.html index.html

for i in $VERTEX_SHADERS
do
    echo "<script id=\"$i-vs\" type=\"x-shader/x-vertex\">" >>index.html
    cat $i.vert >>index.html
    echo "</script>\n" >>index.html
done

for i in $FRAGMENT_SHADERS
do
    echo "<script id=\"$i-fs\" type=\"x-shader/x-fragment\">" >>index.html
    cat $i.frag >>index.html
    echo "</script>\n" >>index.html
done

cat body.html >>index.html
