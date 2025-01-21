#!/bin/bash 
#--------------------------------------------------------------
#   Script: launch.sh
#  Mission: ufld_joust_pavlab
#   Author: Michael Benjamin  
#   LastEd: March 2nd, 2024 
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#--------------------------------------------------------------  
#  Part 2: Set global variables
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
NO_GUI=""
AUTO_LAUNCHED="no"
CMD_ARGS=""
AMT=1
REUSE=""
CIRCLE="x=50,y=-90,rad=40"
START="ring"
COLAVD=""

#--------------------------------------------------------------  
#  Part 3: Check for and handle command-line arguments
#--------------------------------------------------------------  
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                       "
	echo "                                                "
	echo "Options:                                        "
	echo "  --help, -h         Display this help message  "
        echo "  --verbose, -v      Increase verbosity         "
	echo "  --just_make, -j    Just make targs, no launch "
	echo "  --amt=N            Default is 1               " 
	echo "  --nogui, -n        Headless (auto-) launch    "
	echo "                                                "
	echo "  --dock, -d                                    " 
	echo "    Vehicles start at dock instead of on circle " 
	echo "  --colregs, -c                                 " 
	echo "    Use COLREGS avoidance rather than CPA avoid " 
	echo "  --reuse, --norand                             " 
	echo "    Reuse the seed files from the previous run. " 
	echo "  --auto, -a                                    "
	echo "     Auto-launched by a script.                 "
	echo "     Will not launch uMAC as the final step.    "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE=$ARGI
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        NO_GUI=$ARGI
    elif [ "${ARGI}" = "--norand" -o "${ARGI}" = "--reuse" ]; then
	REUSE="--reuse"
    elif [ "${ARGI}" = "--dock" -o "${ARGI}" = "-d" ]; then
	START="dock"
    elif [ "${ARGI}" = "--colregs" -o "${ARGI}" = "-c" ]; then
	COLAVD=$ARGI
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        AMT="${ARGI#--amt=*}"
    else 
        echo "$ME Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

VLAUNCH_ARGS=" --auto $TIME_WARP $VERBOSE $JUST_MAKE"
SLAUNCH_ARGS=" --auto $TIME_WARP $VERBOSE $JUST_MAKE"

VLAUNCH_ARGS+=" $COLAVD "
SLAUNCH_ARGS+=" $NO_GUI "

#----------------z-----------------------------------------------
#  Part 5: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then 
    echo "======================================================"
    echo "              launch.sh SUMMARY                       "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS     = [${CMD_ARGS}]   "
    echo "VERBOSE      = [${VERBOSE}]    "
    echo "JUST_MAKE    = [${JUST_MAKE}]  "
    echo "TIME_WARP    = [${TIME_WARP}]  "
    echo "AMT          = [${AMT}]        "
    echo "START        = [${START}]      "
    echo "REUSE        = [${REUSE}]      "
    echo "COLAVD       = [${COLAVD}]     "
    echo "---------------------------    "
    echo "VLAUNCH_ARGS = [${VLAUNCH_ARGS}] "
    echo "SLAUNCH_ARGS = [${SLAUNCH_ARGS}] "
    echo "---------------------------    "
    echo -n "Hit the RETURN key to continue with launching ALL"
    read ANSWER
fi

#-------------------------------------------------------------
# Part 6: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
pickjoust --circle=$CIRCLE --amt=$AMT --file=joust.txt \
          --spd=1.8,2.2 --ang_min_diff=40 $REUSE       \
	  --ang_max_diff=160 --maxtries=8000 --hdrs 

if [[ ! -f "joust.txt" ]]; then
    echo "Missing Joust.txt file. Exiting."
    exit 1
fi

if [ "${START}" = "dock" ]; then
    pickpos --amt=$AMT --poly=pavlab --file=dock_start.txt  \
	    --hdg=125:185 $REUSE
    
    if [[ ! -f "dock_start.txt" ]]; then
	echo "Missing dock_start.txt file. Exiting."
	exit 1
    fi
fi

#-------------------------------------------------------------
# Part 7: Launch the vehicles
#-------------------------------------------------------------
for IX in `seq 1 $AMT`;
do
    sleep 0.2
    # PX1,PY1 is first vertex of legrun and option for startpos
    PX1=`pluck joust.txt $IX --fld=px1`
    PY1=`pluck joust.txt $IX --fld=py1`
    HDG=`pluck joust.txt $IX --fld=hdg`
    SPD=`pluck joust.txt $IX --fld=spd`
    VNAME=`pluck joust.txt $IX --fld=vname`
    COLOR=`pluck joust.txt $IX --fld=vcolor`
    P1POS="x=${PX1},y=${PY1}"
    
    STAPOS=$P1POS
    if [ "${START}" = "dock" ]; then
	DX1=`pluck dock_start.txt $IX --fld=x`
	DY1=`pluck dock_start.txt $IX --fld=y`
	HDG=`pluck dock_start.txt $IX --fld=heading`	
	STAPOS="x=${DX1},y=${DY1}"
    fi
    
    IX_VLAUNCH_ARGS="$VLAUNCH_ARGS "
    IX_VLAUNCH_ARGS+=" --index=$IX  --vname=$VNAME  "
    IX_VLAUNCH_ARGS+=" --spd=$SPD   --color=$COLOR  " 
    IX_VLAUNCH_ARGS+=" --hdg=$HDG   --start=$STAPOS "

    if [ "$VNAME" = "abe" ]; then
    ./launch_abe.sh $IX_VLAUNCH_ARGS
    else
    ./launch_vehicle.sh $IX_VLAUNCH_ARGS	
    fi
done


#---------------------------------------------------------------
#  Part 8: Launch the shoreside
#---------------------------------------------------------------
echo "$ME: Launching Shoreside ..."
./launch_shoreside.sh $SLAUNCH_ARGS

#---------------------------------------------------------------
#  Part 9: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" -o "${JUST_MAKE}" != "" ]; then
    exit 0
fi

#--------------------------------------------------------------  
#  Part 7: Launch uMAC until mission quit
#--------------------------------------------------------------  
uMAC --paused targ_shoreside.moos
kill -- -$$
