#!/bin/bash -e
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh                                    
#  Mission: ufld_joust
#   Author: Michael Benjamin  
#     Date: Jan 2023
#--------------------------------------------------------------
#  Part 1: Declare global var defaults
#--------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=${TIME_WARP:-1}
JUST_MAKE="no"
VERBOSE=${VERBOSE:-"no"}
AUTO_LAUNCHED=${AUTO_LAUNCHED:-"no"}
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT=${MOOS_PORT:-"9001"}
PSHARE_PORT=${PSHARE_PORT:-"9201"}

SHORE_IP=${SHORE_IP:-"localhost"}
SHORE_PSHARE=${SHORE_PSHARE:-"9200"}
VNAME=${VNAME:-"abe"}
COLOR="yellow"
INDEX=${INDEX:-"1"}

REGION="mit"
VX1_POS=""
VX2_POS=""
START_POS=""
START_HDG=0
SPEED=0
COLAVD="cpa"

#--------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#--------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                            "
	echo "                                                     " 
	echo "Options:                                             "
	echo "  --help, -h         Display this help message       "
        echo "  --verbose, -v      Increase verbosity              "
	echo "  --just_make, -j    Just make targ files, no launch "
	echo "                                                     "
        echo "  --auto, -a                                     "
        echo "     Auto-launched by a script.                  "
        echo "     Will not launch uMAC as the final step.     "
	echo "                                                 "
	echo "  --ip=<localhost>                               " 
	echo "    Force pHostInfo to use this IP Address       "
	echo "  --mport=<9001>                                 "
	echo "    Port number of this vehicle's MOOSDB port    "
	echo "  --pshare=<9201>                                " 
	echo "    Port number of this vehicle's pShare port    "
	echo "                                                 "
	echo "  --shore=<localhost>                            " 
	echo "    IP address location of shoreside             "
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --color=<red>                                  " 
	echo "    Color of the vehicle being launched          " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "  --vx1=<X,Y>     (default is 0,0)               " 
	echo "    vx1 of LegRun chosen by script launching     "
	echo "    this script (to ensure separation)           "
	echo "  --vx2=<X,Y>      (default is 0,0)              " 
	echo "    vx2 of LegRun chosen by script launching     "
	echo "    this script (to ensure separation)           "
	echo "  --start=<X,Y>      (default is 0,0)            " 
	echo "    Ownship start pos for sims chosen by script  "
	echo "    launching this script (to ensure separation) "
	echo "  --hdg=<val>                                    " 
	echo "    Initial vehicle hdg (sim only)               "
	echo "                                                 "
	echo "  --colregs, -c                                 " 
	echo "    Use COLREGS avoidance rather than CPA avoid " 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes" 

    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:7}" = "--mport" ]; then
	MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"

    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"
	
    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:6}" = "--vx1=" ]; then
        VX1_POS="${ARGI#--vx1=*}"
    elif [ "${ARGI:0:6}" = "--vx2=" ]; then
        VX2_POS="${ARGI#--vx2=*}"

    elif [ "${ARGI:0:6}" = "--hdg=" ]; then
        START_HDG="${ARGI#--hdg=*}"
    elif [ "${ARGI:0:6}" = "--spd=" ]; then
        SPEED="${ARGI#--spd=*}"

    elif [ "${ARGI}" = "--colregs" -o "${ARGI}" = "-c" ]; then
	COLAVD="colregs"

    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`
     
#---------------------------------------------------------------
#  Part 3: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then 
    echo "======================================================"
    echo "        launch_vehicle.sh SUMMARY                     "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =      [${CMD_ARGS}]     "
    echo "TIME_WARP =     [${TIME_WARP}]    "
    echo "AUTO_LAUNCHED = [${AUTO_LAUNCHED}]"
    echo "----------------------------------"
    echo "MOOS_PORT =     [${MOOS_PORT}]    "
    echo "PSHARE_PORT =   [${PSHARE_PORT}]  "
    echo "IP_ADDR =       [${IP_ADDR}]      "
    echo "----------------------------------"
    echo "SHORE_IP =      [${SHORE_IP}]     "
    echo "SHORE_PSHARE =  [${SHORE_PSHARE}] "
    echo "VNAME =         [${VNAME}]        "
    echo "COLOR =         [${COLOR}]        "
    echo "INDEX =         [${INDEX}]        "
    echo "----------------------------------"
    echo "VX1_POS   =     [${VX1_POS}]      "
    echo "VX2_POS   =     [${VX2_POS}]      "
    echo "SPEED     =     [${SPEED}]        "
    echo "------------SIM-------------------"
    echo "START_POS =     [${START_POS}]    "
    echo "START_HDG =     [${START_HDG}]    "
    echo "COLAVD =        [${COLAVD}]       "
    echo -n "Hit any key to continue with launching $VNAME"
    read ANSWER
fi

#--------------------------------------------------------------
#  Part 4: Create the .moos and .bhv files. 
#--------------------------------------------------------------
NSFLAGS="-s -f"
if [ "${AUTO}" = "" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME                   \
       START_POS=$START_POS         SHORE_IP=$SHORE_IP             \
       START_HDG=$START_HDG         MOOS_PORT=$MOOS_PORT           \
       SHORE_PSHARE=$SHORE_PSHARE   COLOR=$COLOR                   \
       IP_ADDR=$IP_ADDR   
       
nsplug meta_abe.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME \
       START_POS=$START_POS         SPEED=$SPEED              \
       VX1_POS=$VX1_POS             VX2_POS=$VX2_POS          \
       COLOR=$COLOR                 COLAVD=$COLAVD

if [ ${JUST_MAKE} = "yes" ]; then
    echo "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#--------------------------------------------------------------
#  Part 5: Launch the processes
#--------------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
echo "Done Launching the $VNAME MOOS Community"

#---------------------------------------------------------------
#  Part 6: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 7: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
