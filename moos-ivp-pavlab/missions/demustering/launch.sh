#!/bin/bash -e
# set -x
#---------------------------------------------------------------
#   Script: launch.sh
#  Mission: alpha_heron
#   Author: Mike Benjamin
#   LastEd: 2021-Jun-08
#---------------------------------------------------------------

# Set up a trap to catch termination signals
trap "echo 'Terminating...'; kill -- -$$" SIGINT SIGTERM

#---------------------------------------------------------------
#  Part 1: Set global var defaults
#---------------------------------------------------------------
ME=`basename "$0"`
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
NUM_VEHICLES=8
RANDOM_HEADING=0
VEHICLE_NAMES=("abe" "ben" "cal" "deb" "eve" "fin" "max" "ned" "oak" "pip" "xaa" "xbb" "xcc" "xdd" "xee" "xff" "xgg" "xhh" "xii" "xjj" "xkk" "xll" "xmm" "xnn" "xoo" "xpp")
VEHICLE_START_POS=("4,-4,90" "20,-1,100" "14,-6,240" "27,2,210" "17,0,180" "4,-8,90" "20,-5,100" "14,-10,240" "27,-2,210" "17,-4,180")
BATCH_NAME=""
VLAUNCH_ARGS=""
SLAUNCH_ARGS=""
MISSION_NAME=""
RE_RUN_MISSION=""
TURN_RADIUS="4"




#---------------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#---------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [SWITCHES] [time_warp]                                 "
        echo "  --help, -h         Show this help message                "
        echo "  --just_make, -j    Just make targ files, no launch       "
        echo "  --verbose, -v      Verbose output, confirm before launch "
        echo "  --vnum=<number>    Number of vehicles to launch          "
        echo "  --rand, -r         Randomize vehicle starting positions  "
        echo "  --nogui, -n        Launch without GUI                    "
        echo "  --batch=<name>     Batch name for the mission            "
        echo "  --mission_name=<name> Mission name                       "
        echo "  --rerun=<folder>   Re-run a mission                      "
        echo "  --turnRadius=<radius> Turn radius for vehicles           "
        echo "  --projectFirst    Project 1st points in front of the veh "
        echo "  --assAlg=<algorithm> Assignment algorithm                "
        echo "  --assMet=<metric> Assignment metric                      "
        echo "  --turnInPlace     Turn in place                          "
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [[ "${ARGI}" =~ --vnum=([0-9]+) ]]; then
        NUM_VEHICLES=${BASH_REMATCH[1]}
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="-j"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="--verbose"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RANDOM_HEADING=1
    elif [ "${ARGI:0:8}" = "--batch=" ]; then
        BATCH_NAME="${ARGI#--batch=*}"
    elif [ "${ARGI:0:15}" = "--mission_name=" ]; then
        MISSION_NAME="${ARGI#--mission_name=*}"
    elif [ "${ARGI:0:8}" = "--rerun=" ]; then
        RE_RUN_MISSION="${ARGI#--rerun=*}"
    elif [ "${ARGI:0:13}" = "--turnRadius=" ]; then
        TURN_RADIUS="${ARGI#--turnRadius=*}"
    elif [ "${ARGI:0:15}" = "--projectFirst=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--assAlg=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--assMet=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:17}" = "--headDistWeight=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:17}" = "--tempBlockLimit=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:14}" = "--turnInPlace=" ]; then
        SLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:18}" = "--useDynamicSpeed=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:9}" = "--dthMin=" ]; then
        VLAUNCH_ARGS+=" $ARGI"
    else
        echo "$ME: Bad arg:" $ARGI "Exit Code 1."
        exit 1
    fi
done

# Ensure the number of vehicles does not exceed the length of the VEHICLE_NAMES array
if [ "$NUM_VEHICLES" -gt "${#VEHICLE_NAMES[@]}" ]; then
    echo "Error: Number of vehicles exceeds the number of available vehicle names."
    exit 1
fi

if [ "$RANDOM_HEADING" -eq 1 ]; then
    x_min=0
    x_max=40
    y_min=-35
    y_max=-15

    min_distance=4

    VEHICLE_START_POS=()

    generate_random_number() {
        local min=$1
        local max=$2
        echo $((RANDOM % (max - min + 1) + min))
    }
    generate_random_heading() {
        echo $((RANDOM % 361))
    }
    calculate_distance() {
        local x1=$1
        local y1=$2
        local x2=$3
        local y2=$4
        echo "sqrt(($x2 - $x1) * ($x2 - $x1) + ($y2 - $y1) * ($y2 - $y1))" | bc -l
    }
    # TODO: Look into pickpos
    # Loop to generate random coordinates and headings
    for ((i = 0; i < NUM_VEHICLES; i++)); do
        echo "$i"
        while true; do
            # Generate random x and y coordinates
            x=$(generate_random_number $x_min $x_max)
            y=$(generate_random_number $y_min $y_max)
            # Check if the new position is at least 1 meter away from all previous positions
            valid=true
            for pos in "${VEHICLE_START_POS[@]}"; do
                IFS=',' read -r px py _ <<< "$pos"
                distance=$(calculate_distance $x $y $px $py)
                if (( $(echo "$distance < $min_distance" | bc -l) )); then
                    valid=false
                    break
                fi
            done
            if $valid; then
                # Generate a random heading between 0 and 360
                heading=$(generate_random_heading)
                # Combine the coordinates with the heading
                start_pos="${x},${y},${heading}"
                # Add the start position to the array
                VEHICLE_START_POS+=("${start_pos}")
                break
            fi
        done
    done


# if [ "$RANDOM_HEADING" -eq 1 ]; then
#     anchor_x=20  # Center of the formation
#     anchor_y=-25 # Center of the formation
#     initial_radius=40  # Starting radius
#     radius_increment=10
#     min_distance=3
#     max_attempts=10

#     VEHICLE_START_POS=()

#     generate_random_heading() {
#         echo $((RANDOM % 361))
#     }
#     calculate_distance() {
#         local x1=$1
#         local y1=$2
#         local x2=$3
#         local y2=$4
#         echo "sqrt(($x2 - $x1) * ($x2 - $x1) + ($y2 - $y1) * ($y2 - $y1))" | bc -l
#     }

#     # generate_random_float() {
#     #     echo "$((RANDOM % 1001)) / 1000" | bc -l
#     # }

#     generate_random_float() {
#         # Use bc to ensure floating-point division
#         echo "scale=4; $RANDOM / 32767" | bc
#     }

#     # Usage in loop
#     radius=$initial_radius

#     # # Generate and echo 100 random floats and 100 random angles
#     # radius_test=10
#     # for i in {1..100}; do
#     #     # echo "sqrt($(generate_random_float)) * $radius_test" | bc -l
#     #     echo "$(generate_random_float) * 2 * 3.14159265358979323846" | bc -l
#     # done

#     # # exit 1



#     for ((i = 0; i < NUM_VEHICLES; i++)); do
#         attempts=0
#         while true; do
#             # Generate x and y using the function
#             random_angle=$(echo "$(generate_random_float) * 2 * 3.14159265358979323846" | bc -l)
#             random_distance=$(echo "sqrt($(generate_random_float)) * $radius" | bc -l)  
#             # echo "Random distance: $random_distance"
#             # echo "Random angle: $random_angle"

#             random_x=$(awk -v ax="$anchor_x" -v d="$random_distance" -v a="$random_angle" 'BEGIN { print ax + d * cos(a) }')
#             random_y=$(awk -v ay="$anchor_y" -v d="$random_distance" -v a="$random_angle" 'BEGIN { print ay + d * sin(a) }')

#             echo "Random x: $random_x"
#             echo "Random y: $random_y"

#             # Validate that the position is sufficiently far from all previous ones
#             valid=true
#             for pos in "${VEHICLE_START_POS[@]}"; do
#                 IFS=',' read -r px py _ <<< "$pos"
#                 dist=$(calculate_distance "$random_x" "$random_y" "$px" "$py")
#                 if (( $(echo "$dist < $min_distance" | bc -l) )); then
#                     valid=false
#                     break
#                 fi
#             done

#             if $valid; then
#                 # Generate a random heading and store the position
#                 heading=$(generate_random_heading)
#                 VEHICLE_START_POS+=("${random_x},${random_y},${heading}")
#                 break
#             else
#                 # Increment radius if attempts exceed threshold
#                 attempts=$((attempts + 1))
#                 if (( attempts > max_attempts )); then
#                     radius=$(echo "$radius + $radius_increment" | bc -l)
#                     echo "Increasing radius to $radius"
#                     attempts=0
#                 fi
#             fi
#         done
#     done

elif [ -n "$RE_RUN_MISSION" ]; then    
    echo "$ME: Re-running mission $RE_RUN_MISSION"
    VEHICLE_START_POS=()

    rm -rf $RE_RUN_MISSION/temp_nav_logs
    mkdir -p $RE_RUN_MISSION/temp_nav_logs
    
    # Loop through all subfolders in the RE_RUN_MISSION folder and echo the *.alog filename
    COUNTER=0
    for folder in "$RE_RUN_MISSION"/*; do
        if [ -d "$folder" ]; then
            if [[ "$folder" == *SHORE* ]]; then
                continue
            elif [[ "$folder" == *temp_nav_logs* ]]; then
                continue
            fi

            # Check if there is an .alog file in the folder
            for alog_file in "$folder"/*.alog; do
                if [ -f "$alog_file" ]; then
                    let COUNTER=COUNTER+1
                    alogpath=$(realpath $alog_file)
                    aloggrep ${alogpath} NAV_X NAV_Y NAV_HEADING -sd $RE_RUN_MISSION/temp_nav_logs/${COUNTER}.alog 1> /dev/null
                    alogpath_pruned=$RE_RUN_MISSION/temp_nav_logs/${COUNTER}.alog

                    first_nav_x=""
                    first_nav_y=""
                    first_nav_heading=""

                    while IFS= read -r line; do
                        time=$(echo "$line" | awk '{print $1}')
                        variable=$(echo "$line" | awk '{print $2}')
                        value=$(echo "$line" | awk '{print $4}')

                        if [[ "$variable" == "NAV_X" && -z "$first_nav_x" ]]; then
                            first_nav_x=$value
                        elif [[ "$variable" == "NAV_Y" && -z "$first_nav_y" ]]; then
                            first_nav_y=$value
                        elif [[ "$variable" == "NAV_HEADING" && -z "$first_nav_heading" ]]; then
                            first_nav_heading=$value
                        fi

                        # Break if all variables are found
                        if [[ -n "$first_nav_x" && -n "$first_nav_y" && -n "$first_nav_heading" ]]; then
                            break
                        fi
                    done < <(tail -n +5 "$alogpath_pruned")  # Skip the first 4 lines of the file

                    start_pos="${first_nav_x},${first_nav_y},${first_nav_heading}"
                    VEHICLE_START_POS+=("${start_pos}")
                fi
            done
        fi
    done

    rm -rf $RE_RUN_MISSION/temp_nav_logs
    NUM_VEHICLES=$COUNTER
fi



#---------------------------------------------------------------
#  Part 3: Initialize and Launch the vehicles
#---------------------------------------------------------------

# Unique mission name
# if mission name is not provided, generate a unique one
if [ -z "$MISSION_NAME" ]; then
    MISSION_NAME=$(date +"%y%m%d-%H%M%S")
fi

echo "$ME: Mission Name is $MISSION_NAME"
echo "$ME: Batch Name is $BATCH_NAME"
mkdir -p ./logs_simulation/$BATCH_NAME/$MISSION_NAME

for ((i=1; i<=NUM_VEHICLES; i++))
do
    echo "$i"
    VNAME=${VEHICLE_NAMES[$((i-1))]}
    VPOS=${VEHICLE_START_POS[$((i-1))]}
    VLAUNCH_ARGS_COPY=$VLAUNCH_ARGS
    VLAUNCH_ARGS_COPY+=" --auto --noconfirm --sim --vname=$VNAME --index=$i --start=$VPOS --mission_name=$MISSION_NAME --batch=$BATCH_NAME"
    VLAUNCH_ARGS_COPY+=" --use_compass=false"
    VLAUNCH_ARGS_COPY+=" --tr=$TURN_RADIUS"
    # VLAUNCH_ARGS_COPY+=" --shore=192.168.1.10" #TEMP
    echo "$ME: Launching $VNAME ..."
    ./launch_vehicle.sh $VLAUNCH_ARGS_COPY $VERBOSE $JUST_MAKE $TIME_WARP
done



#---------------------------------------------------------------
#  Part 4: Launch the shoreside
#---------------------------------------------------------------
echo "$ME: Launching Shoreside ..."
SLAUNCH_ARGS+=" --tr=$TURN_RADIUS"
# SLAUNCH_ARGS+=" --ip=192.168.1.10" #TEMP
./launch_shoreside.sh --auto --noconfirm $VERBOSE $JUST_MAKE $TIME_WARP --mission_name=$MISSION_NAME --batch=$BATCH_NAME --sim $SLAUNCH_ARGS

#---------------------------------------------------------------
# Part 5: Launch uMAC until the mission is quit
#---------------------------------------------------------------
uMAC targ_shoreside.moos
kill -- -$$