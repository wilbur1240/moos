#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch.sh     
#  Mission: convoy_ctx
#   Author: Michael Benjamin   
#   LastEd: November 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
AUTO_LAUNCHED="no"
CMD_ARG=""
NOGUI=""
NOCONFIRM="-nc"

RANDSTART="true"
AMT=4
CNAMT=0

TRANSIT_SPD=3.0
MAXIMUM_SPD=5.0

CONVOY_VERS=""
MUSTER_REGION="-1400,100:-1400,-100:-1200,-100:-1200,100"

VLAUNCH_ARGS=" --auto "
SLAUNCH_ARGS=" --auto "
CLEAN="no"

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                               "
	echo "                                                        "
        echo "Synopsis:                                               "
	echo "  A script for launching an arbitrary number of vehicles"
	echo "                                                        "
	echo "Options:                                                "
        echo "  --help, -h                                            "
        echo "    Display this help message                           "
        echo "  --verbose, -v                                         "
        echo "    Increase verbosity                                  "
	echo "  --just_make, -j                                       " 
	echo "    Just make the targ files, but do not launch.        " 
	echo "  --amt=N                                               " 
	echo "    The number of vehicles to launch. The default is 1. "
	echo "  --cn=N                                                " 
	echo "    The number of contacts to launch. The default is 0. "
	echo "  --active_convoy, -ac                                  " 
	echo "    Use Active Convoy Behavior: BHV_ConvoyV21X          "
	echo "  --clean, -cc                                          " 
	echo "    Run clean.sh and ktm prior to launch                "
	echo "  --norand                                              " 
	echo "    Do not randomly generate files vpositions.txt. Just "
	echo "    re-use the previous versions if they exist.         "
	echo "                                                        "
	echo "Options For launches on MTASC machines:                 "
	echo "                                                        "
	echo "  --shore=<ipaddr>                                      " 
	echo "    IP address where nodes can expect to find shoreside "
	echo "                                                        "
	exit 0;
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE="--verbose"
	NOCONFIRM=""
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	VLAUNCH_ARGS+=" $ARGI"
	SLAUNCH_ARGS+=" $ARGI"
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--logclean" -o "${ARGI}" = "-l" ]; then
	VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--norand" -o "${ARGI}" = "-r" ]; then
	RANDSTART="false"
    elif [ "${ARGI}" = "--mtasc" -o "${ARGI}" = "-m" ]; then
	MTASC="yes"
    elif [ "${ARGI}" = "--clean" -o "${ARGI}" = "-cc" ]; then
	CLEAN="yes"
    elif [ "${ARGI}" = "--auto" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
	VLAUNCH_ARGS=" $ARGI"
    elif [ "${ARGI}" = "--cache" -o "${ARGI}" = "-c" ]; then
        USE_CACHE="--cache"
    elif [ "${ARGI}" = "--active_convoy" -o "${ARGI}" = "-ac" ]; then
	CONVOY_VERS="--active_convoy"
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        AMT="${ARGI#--amt=*}"
	if [ ! $AMT -ge 1 ]; then
	    echo "$ME: Vehicle amount must be >= 1."
	    exit 1
	fi
    elif [ "${ARGI:0:5}" = "--cn=" ]; then
        CNAMT="${ARGI#--cn=*}"
	if [ ! $CNAMT -ge 1 ]; then
	    echo "$ME: contact amount must be >= 0."
	    exit 1
	fi
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then 
    echo "======================================================"
    echo "              launch.sh SUMMARY                       "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =       [${CMD_ARGS}]      "
    echo "TIME_WARP =      [${TIME_WARP}]     "
    echo "AUTO_LAUNCHED =  [${AUTO_LAUNCHED}] "
    echo "AMT =            [${AMT}]           "
    echo "CNAMT =          [${CNAMT}]         "
    echo "----------------------------------  "
    echo "TRANSIT_SPD =    [${TRANSIT_SPD}]   "
    echo "MAXIMUM_SPD =    [${MAXIMUM_SPD}]   "
    echo "CONVOY_VERS =    [${CONVOY_VERS}]   "
    echo "MUSTER_REGION =  [${MUSTER_REGION}] "
    echo -n "Hit the RETURN key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------------
# Part 5A: If Cleaning enabled, clean first
#-------------------------------------------------------------
vecho "Running ./clean.sh and ktm prior to launch"
if [ "${CLEAN}" = "yes" ]; then
    ./clean.sh; ktm
fi

#-------------------------------------------------------------
# Part 5B: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
vecho "Picking Convoy vehicle starting positions."

if [ "${RANDSTART}" = "true" -o  ! -f "vpositions.txt" ]; then
    ./pickpos.sh $AMT
fi

# vehicle names and colors are always deterministic
pickpos --amt=$AMT --vnames  > vnames.txt
pickpos --amt=$AMT --colors  > vcolors.txt

VEHPOS=(`cat vpositions.txt`)
VNAMES=(`cat vnames.txt`)
COLORS=(`cat vcolors.txt`)

#-------------------------------------------------------------
# Part 5C: Launch the convoy vehicles
#-------------------------------------------------------------
ALL_VNAMES=""
for INDEX in `seq 1 $AMT`;
do
    sleep 0.2
    ARRAY_INDEX=`expr $INDEX - 1`
    START=${VEHPOS[$ARRAY_INDEX]}
    VNAME=${VNAMES[$ARRAY_INDEX]}
    COLOR=${COLORS[$ARRAY_INDEX]}

    if [ "${ALL_VNAMES}" != "" ]; then
	ALL_VNAMES+=":"
    fi
    ALL_VNAMES+=$VNAME
    
    VLAUNCH_ARGS+=" $NOCONFIRM "
    IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
    IX_VLAUNCH_ARGS+=" --muster_region=$MUSTER_REGION    "
    IX_VLAUNCH_ARGS+=" --index=$INDEX --start=$START     "
    IX_VLAUNCH_ARGS+=" --maxspd=$MAXIMUM_SPD             "
    IX_VLAUNCH_ARGS+=" --speed=$TRANSIT_SPD              " 
    IX_VLAUNCH_ARGS+=" --color=$COLOR                    " 
    IX_VLAUNCH_ARGS+=" --vname=$VNAME  --shore=localhost "
    IX_VLAUNCH_ARGS+=" $VERBOSE $CONVOY_VERS $TIME_WARP  "

    #vecho "Launching: $VNAME"
    #vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"

    ./launch_vehicle.sh $IX_VLAUNCH_ARGS	
done

#-------------------------------------------------------------
# Part 6A: Generate random starting positions, speeds and vnames
#-------------------------------------------------------------
if [ "$CNAMT" -gt 0 ]; then
    vecho "Picking CONTACT start positions and names..."
    ./pickpos_cn.sh $CNAMT > vpositions_cn.txt

    VEHPOS=(`cat vpositions_cn.txt`)
    VNAMES=(`cat vnames_cn.txt`)
fi

#-------------------------------------------------------------
# Part 6B: Launch the CONTACT vehicles
#-------------------------------------------------------------
if [ "$CNAMT" -gt 0 ]; then
    for INDEX in `seq 1 $CNAMT`;
    do
	sleep 0.2
	
	ARRAY_INDEX=`expr $INDEX-1`
	START=${VEHPOS[$ARRAY_INDEX]}
	VNAME=${VNAMES[$ARRAY_INDEX]}

	if [ "${ALL_VNAMES}" != "" ]; then
	    ALL_VNAMES+=":"
	fi
	ALL_VNAMES+=$VNAME
    
	VLAUNCH_ARGS+=" $NOCONFIRM "
	IX_VLAUNCH_ARGS=$VLAUNCH_ARGS
	IX_VLAUNCH_ARGS+=" --index=$INDEX --start=$START     "
	IX_VLAUNCH_ARGS+=" --maxspd=5.0 --speed=3.5          "
	IX_VLAUNCH_ARGS+=" --color=dodger_blue               " 
	IX_VLAUNCH_ARGS+=" --vname=$VNAME  --shore=localhost "
	IX_VLAUNCH_ARGS+=" $VERBOSE $TIME_WARP  "
	
	vecho "Launching: $VNAME"
	vecho "IX_VLAUNCH_ARGS: [$IX_VLAUNCH_ARGS]"
	
	./launch_vehicle_cn.sh $IX_VLAUNCH_ARGS	
    done
fi

#-------------------------------------------------------------
# Part 7: Launch the Shoreside mission file
#-------------------------------------------------------------
vecho "Launching the shoreside. Args: $SLAUNCH_ARGS $TIME_WARP"

SLAUNCH_ARGS+=" --vnames=$ALL_VNAMES --muster_region=$MUSTER_REGION"

./launch_shoreside.sh $SLAUNCH_ARGS $VERBOSE $TIME_WARP
sleep 0.25

#---------------------------------------------------------------
#  Part 8: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" -o "${JUST_MAKE}" != "" ]; then
    exit 0
fi

#-------------------------------------------------------------
# Part 9: Launch uMAC in paused mode until the mission is quit
#-------------------------------------------------------------
uMAC --paused targ_shoreside.moos
kill -- -$$
