#!/bin/bash -v
git add --all .
git commit
git push -u origin $1
