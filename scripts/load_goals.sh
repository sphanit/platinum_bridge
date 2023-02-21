#! /usr/bin/env bash
result=$(rospack find platinum_bridge)
cd $result/scripts
python loadGoals.py