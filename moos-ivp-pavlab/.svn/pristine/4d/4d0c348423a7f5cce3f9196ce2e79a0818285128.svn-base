#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: pavop.sh    
#   Author: Michael Benjamin   
#   LastEd: July 2023
#-------------------------------------------------------------- 
#  Part 1: Set global var defaults
#-------------------------------------------------------------- 
ME=`basename "$0"`
WID=100
DEP=60
NEX=60
NEY=10
ANG="64.64"
ANG_90="154.64"
ANG_180="244.64"

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS]                        " 
        echo "                                                       "
        echo "Synopsis:                                              "
        echo "  $ME will generate corners of a rectangular op region "
        echo "  based on base corner. The defaults are based on the  "
        echo "  MIT Sailing Pavilion, but can be used for any area   "
        echo "  by overriding the defaults.                          "
        echo "  The default base corner is the NE corner. Default    "
        echo "  base angle is 64.64 degrees which represents the     "
        echo "  the angle from the NW to NE corner. 64.64 degrees    "
        echo "  happens to be the relative angle from the Western    "
        echo "  corner of the Pavilion dock to the Eastern corner.   "
        echo "                                                       "
        echo "Options:                                               "
        echo "  --help, -h                                           "
        echo "    Display this help message                          "
        echo "  --wid=N                                              "
        echo "    Width of oparea (aligned w/ dock) (Default 100)    "
        echo "  --dep=N                                              "
        echo "    Depth of oparea (perp to dock) (Default 60)        "
        echo "  --nex=N                                              "
        echo "    Northeast corner x-coordinate (Default 60)         "
        echo "  --ney=N                                              "
        echo "    Northeast corner y-coordinate (Default 10)         "
        echo "                                                       "
        echo "Examples:                                              "
        echo " $ ./pavop.sh                                          "
        echo " $ ./pavop.sh --wid=150 --dep=90                       "
        echo " $ ./pavop.sh --nex=70 --ney=20                        "
	exit 0;
    elif [ "${ARGI:0:6}" = "--wid=" ]; then
        WID="${ARGI#--wid=*}/"
    elif [ "${ARGI:0:6}" = "--dep=" ]; then
        DEP="${ARGI#--dep=*}/"
    elif [ "${ARGI:0:6}" = "--nex=" ]; then
        NEX="${ARGI#--nex=*}/"
    elif [ "${ARGI:0:6}" = "--ney=" ]; then
        NEY="${ARGI#--ney=*}/"
    else
        echo "$ME Bad Arg:" $ARGI " Exit Code 1"
        exit 1
    fi
done

NWX=`projpt $NEX $NEY  $ANG_180 $WID | cut -d" " -f1`
NWY=`projpt $NEX $NEY  $ANG_180 $WID | cut -d" " -f2`

SWX=`projpt $NWX $NWY  $ANG_90 $DEP | cut -d" " -f1`
SWY=`projpt $NWX $NWY  $ANG_90 $DEP | cut -d" " -f2`

SEX=`projpt $NEX $NEY  $ANG_90 $DEP | cut -d" " -f1`
SEY=`projpt $NEX $NEY  $ANG_90 $DEP | cut -d" " -f2`

echo "pts={$NEX,$NEY:$NWX,$NWY:$SWX,$SWY:$SEX,$SEY}"
