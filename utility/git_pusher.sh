#!/bin/bash
for f in */; do
    echo $f
    cd $f
    `git push -q`
    echo "------------------------"
    cd ..
done
