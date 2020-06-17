#!/bin/bash
for f in */; do
    echo $f
    cd $f
    `git pull -q`
    echo "------------------------"
    cd ..
done
