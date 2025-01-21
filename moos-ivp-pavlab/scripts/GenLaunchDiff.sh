#!/bin/bash
#--------------------------------------------------
#   Script: GenlaunchDiff.sh
#   Author: M.Benjamin
#   LastEd: May 2024
#--------------------------------------------------
for ARGI; do
    CMD_ARGS+="${ARGI} "
done

FILE="launch.sh"

GenLaunch.sh $CMD_ARGS --output=tmp_$FILE
diff $FILE tmp_$FILE
rm -f tmp_$FILE

exit 0
