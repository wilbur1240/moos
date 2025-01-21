#!/bin/bash
#-------------------------------------------------------------- 
#   Script: launch_vehicle.sh    
#  Mission: convoy_ct
#   Author: Michael Benjamin   
#   LastEd: November 2021
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#-------------------------------------------------------------- 
#  Part 2: Set Global Variables
#-------------------------------------------------------------- 
# Modified by Supun on 22/06/2021

ME=`basename "$0"`
TIME_WARP=1
VERBOSE=""
JUST_MAKE="no"
CONFIRM="yes"
AUTO_LAUNCHED="no"
CMD_ARGS=""

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"

SHORE_IP="192.168.1.224"
SHORE_PSHARE="9200"
VNAME=""
COLOR="gray70"
INDEX="1"

REGION="pavlab"
MUSTER_REGION="-1400,100:-1400,-100:-1200,-100:-1200,100"
START_POS="0,0,180"  
RETURN_POS="5,0"
TRANSIT_SPD="1.0"  
MAXIMUM_SPD="2.0"

CONVOY_VERS="new"

#-------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#-------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME: [OPTIONS] [time_warp]                       "
	echo "                                                 "
	echo "Options:                                         "
	echo "  --help, -h                                     " 
	echo "    Print this help message and exit             "
	echo "  --just_make, -j                                " 
	echo "    Just make targ files, but do not launch      "
	echo "  --verbose, -v                                  " 
	echo "    Verbose output, confirm before launching     "
	echo "  --noconfirm, -nc                               " 
	echo "    No confirmation before launching             "
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
	echo "  --shore_pshare=<9200>                          " 
	echo "    Port on which shoreside pShare is listening  "
	echo "  --vname=<abe>                                  " 
	echo "    Name of the vehicle being launched           " 
	echo "  --color=<color>                                " 
	echo "    Color of the vehicle being launched          " 
	echo "  --index=<1>                                    " 
	echo "    Index for setting MOOSDB and pShare ports    "
	echo "                                                 "
	echo "  --muster_region=Polygon                        " 
	echo "    Set the polygon muster region                "
	echo "  --start=<X,Y,H>     (default is 0,0,180)       " 
	echo "    Start position chosen by script launching    "
	echo "    this script (to ensure separation)           "
	echo "  --speed=meters/sec                             " 
	echo "    The speed use for transiting/loitering       "
	echo "                                                 "
	echo "  --maxspd=meters/sec                            " 
	echo "    Max speed of vehicle (for sim and in-field)  "
        echo "  --mtasc, -m                                    "
        echo "     Mission is remotely launched (e.g. MTASC)   "
        echo "  --active_convoy, -ac                           "
        echo "     Use Active Convoy Behavior: BHV_ConvoyV21X  "
	echo "                                                 "
	echo "  --gus,   -G  : Gus vehicle.                    "
	echo "  --hal,   -H  : Hal vehicle.                    "
	echo "  --ida,   -I  : Ida vehicle.                    "
	echo "  --jing,  -J  : Jing vehicle.                   "
	echo "  --m300       : Use iM300 interface             "
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
	JUST_MAKE="yes"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--noconfirm" -o "${ARGI}" = "-nc" ]; then
	CONFIRM="no"
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
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:8}" = "--index=" ]; then
        INDEX="${ARGI#--index=*}"
    elif [ "${ARGI:0:16}" = "--muster_region=" ]; then
        MUSTER_REGION="${ARGI#--muster_region=*}"

    elif [ "${ARGI:0:8}" = "--start=" ]; then
        START_POS="${ARGI#--start=*}"
    elif [ "${ARGI:0:8}" = "--speed=" ]; then
        TRANSIT_SPD="${ARGI#--speed=*}"
    elif [ "${ARGI:0:9}" = "--maxspd=" ]; then
        MAXIMUM_SPD="${ARGI#--maxspd=*}"

    elif [ "${ARGI}" = "--active_convoy" -o "${ARGI}" = "-ac" ]; then
	CONVOY_VERS="active"

    else 
	echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
	exit 1
    fi
done

MOOS_PORT=`expr $INDEX + 9000`
PSHARE_PORT=`expr $INDEX + 9200`

#---------------------------------------------------------------
#  Part 4: If verbose, show vars and confirm before launching
#---------------------------------------------------------------
if [ "${VERBOSE}" = "yes" -o "${CONFIRM}" = "yes" ]; then 
    echo "======================================================"
    echo "         launch_vehicle.sh SUMMARY               $VNAME "
    echo "======================================================"
    echo "$ME"
    echo "CMD_ARGS =       [${CMD_ARGS}]      "
    echo "TIME_WARP =      [${TIME_WARP}]     "
    echo "AUTO_LAUNCHED =  [${AUTO_LAUNCHED}] "
    echo "----------------------------------  "
    echo "MOOS_PORT =      [${MOOS_PORT}]     "
    echo "PSHARE_PORT =    [${PSHARE_PORT}]   "
    echo "IP_ADDR =        [${IP_ADDR}]       "
    echo "----------------------------------  "
    echo "SHORE_IP =       [${SHORE_IP}]      "
    echo "SHORE_PSHARE =   [${SHORE_PSHARE}]  "
    echo "VNAME =          [${VNAME}]         "
    echo "COLOR =          [${COLOR}]         "
    echo "INDEX =          [${INDEX}]         "
    echo "----------------------------------  "
    echo "START_POS =      [${START_POS}]     "
    echo "TRANSIT_SPD =    [${TRANSIT_SPD}]   "
    echo "MAXIMUM_SPD =    [${MAXIMUM_SPD}]   "
    echo "CONVOY_VERS =    [${CONVOY_VERS}]   "
    echo "MUSTER_REGION =  [${MUSTER_REGION}] "
    echo -n "Hit the RETURN key to continue with launching"
    read ANSWER
fi

#-------------------------------------------------------
#  Part 5: Create the .moos and .bhv files. 
#-------------------------------------------------------
# interactive nsplug only when not being remotely launched.
NSFLAGS="-s -f"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="-i -f"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP  \
       PSHARE_PORT=$PSHARE_PORT     VNAME=$VNAME                 \
       COLOR=$COLOR                 MAXSPD=$MAXSPD               \
       START_POS=$START_POS         SHORE_IP=$SHORE_IP           \
       SHORE_PSHARE=$SHORE_PSHARE   MOOS_PORT=$MOOS_PORT         \
       IP_ADDR=$IP_ADDR             MAXIMUM_SPD=$MAXIMUM_SPD     \
       START_POS=$START_POS         
       
nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSFLAGS VNAME=$VNAME    \
       TRANSIT_SPD=$TRANSIT_SPD     COLOR=$COLOR                 \
       CONVOY_VERS=$CONVOY_VERS     MUSTER_REGION=$MUSTER_REGION \
       COLOR=$COLOR
       
if [ "${JUST_MAKE}" = "yes" ]; then
    vecho "Files assembled; nothing launched; exiting per request."
    exit 0
fi

#-------------------------------------------------------
#  Part 6: Launch the vehicle mission
#-------------------------------------------------------
vecho "Launching $VNAME MOOS Community. WARP="$TIME_WARP
pAntler targ_$VNAME.moos >& /dev/null &
vecho "Done Launching $VNAME MOOS Community"

#---------------------------------------------------------------
#  Part 7: If launched from script, we're done, exit now
#---------------------------------------------------------------
if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

#---------------------------------------------------------------
# Part 8: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_$VNAME.moos
kill -- -$$
