#/bin/bash

# This small script is removing all files with the extensions commonly used for RHBP logs
# from the .ros directory of the current user.

cd ~/.ros

find -name \*.pddl -delete
find -name \*.log -delete

