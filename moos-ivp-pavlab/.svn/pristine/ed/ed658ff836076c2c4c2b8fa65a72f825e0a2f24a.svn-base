#!/bin/bash -e
# set -x
#--------------------------------------------------------------
#   Script: blaunch.sh  (Batch Launch)
#   Author: Filip Stromstad
#     Date: 2024-Aug-30
#--------------------------------------------------------------


TIME_WARP=20
MISSION_TIME=400
#If a MOOS process is no longer reporting an uptime, in realtime, we track the process time to bring things down just in case
PROCESS_TIME=$(( 10 + $MISSION_TIME / $TIME_WARP))
TRIALS=10
POST_PROCESS_SCRIPT_PATH="" 
LAUNCH_ARGS=""

idx=0
while [[ idx -lt $# ]]; do
    idx=$((idx+1))
    ARGI=${!idx}
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "Not yet!"
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI:0:4}" = "--t=" ]; then
        TRIALS="${ARGI#--t=*}"
    elif [ "${ARGI}" = "--nogui" -o "${ARGI}" = "-n" ]; then
        LAUNCH_ARGS+=" $ARGI"
    elif [ "${ARGI:0:7}" = "--vnum=" ]; then
        LAUNCH_ARGS+=" $ARGI"
    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

CONFIG_FILE="config.txt"
CONFIGS=()


# Read through each line in the file "config.txt" and generate the configuration conditions
while IFS= read -r line || [[ -n "$line" ]]; do
    # Each line is on the form: "--var_name=[start:step:end]"
    # If first character is not a "-", skip the line
    if [ "${line:0:1}" != "-" ]; then
        continue
    fi

    if [[ "$line" == *"["*"]"* ]]; then
        var_name=$(echo "$line" | cut -d '=' -f 1)
        range=$(echo "$line" | cut -d '[' -f 2 | cut -d ']' -f 1)
        
        start=$(echo "$range" | cut -d ':' -f 1)
        step=$(echo "$range" | cut -d ':' -f 2)
        end=$(echo "$range" | cut -d ':' -f 3)

        i=$start    
        while (( $(echo "$i <= $end" | bc -l) )); do
            CONFIGS+=("${var_name}=$i")
            i=$(echo "$i + $step" | bc -l)
        done
    else
        CONFIGS+=("$line")
    fi
done < "$CONFIG_FILE"

# # Output the generated configuration conditions
# for config in "${CONFIGS[@]}"; do
#     echo "$config"
# done

# Verify the input parameters before staring the batch
echo "You are about to run a batch with:"
echo "  - ${#CONFIGS[@]} configurations"
echo "  - ${TRIALS} trials per configuration"
echo "  - For a total of $(( ${#CONFIGS[@]} * ${TRIALS} )) missions"
echo "  - This will take approximately $(( ${#CONFIGS[@]} * ${TRIALS} * $MISSION_TIME / $TIME_WARP / 60 )) minutes ($(( ${#CONFIGS[@]} * ${TRIALS} * $MISSION_TIME / $TIME_WARP / 3600 )) hours)"
echo "  - launch args: $LAUNCH_ARGS"
echo "Are you sure you want to continue? [y/n]"
read -r response
if [ "$response" != "y" ]; then
    echo "Exiting..."
    exit 0
fi



idx=0
config_idx=0
t_start=$(date +%s)
ktm

BATCH_NAME="batch_$(date +"%y%m%d-%H%M")"
mkdir -p ./logs_simulation/$BATCH_NAME
mkdir -p ./logs_simulation/$BATCH_NAME/failed_attempts
cp $CONFIG_FILE ./logs_simulation/$BATCH_NAME


# Get the name of the first config
FIRST_CONFIG=""


for CONFIG in "${CONFIGS[@]}"; do
    config_idx=$((config_idx+1))
    #get the config name without the preceeding "--" and replace the "=" with "_"
    config_name=$(echo $CONFIG | cut -c 3- | sed 's/=/_/g')

    if [ $config_idx -eq 1 ]; then
        FIRST_CONFIG=$config_name
    fi

    trial_idx=0
    failed_attempts=0
    # for k in $(seq 1 $TRIALS); do
    while [ $trial_idx -lt $TRIALS ]; do
        idx=$((idx+1))
        trial_idx=$((trial_idx+1))
        # ktm
        #Set timers
        mission_duration=0
        mission_start=$(date +%s)
        mission_name=$(date +"%y%m%d-%H%M%S")_${config_name}_trial_${trial_idx}
        mission_failed=false

        t_now=$(date +%s)
        duration=$((t_now-t_start))

        percent_done=$(echo "scale=2; $idx / (${#CONFIGS[@]} * ${TRIALS}) * 100" | bc -l)
        
        echo "${percent_done}% done: config ${config_idx}/${#CONFIGS[@]} (${CONFIG}), trial ${trial_idx}/${TRIALS} - ${duration} seconds elapsed"

        #Run the mission and detach, but capture the Process ID number
        # ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rand --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &

        # if this is the first CONFIG...
        if [ $config_idx -eq 1 ]; then
            ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rand --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &
        else
            FIRST_CONFIG_TRIAL=$(ls -d logs_simulation/$BATCH_NAME/*_"${FIRST_CONFIG}"_trial_"${trial_idx}")
            # echo "Copying $FIRST_CONFIG_TRIAL to $mission_name"
            ./launch.sh $LAUNCH_ARGS $CONFIG $TIME_WARP --rerun=$FIRST_CONFIG_TRIAL --batch=$BATCH_NAME --mission_name=$mission_name&> /dev/null &
        fi

        pid_l=$!
        disown $pid_l # Detach the process to avoid the 'Terminated' message
        
        #Let things bring themselves up
        sleep 10

        #Start the mission by poking the DB
        uPokeDB targ_shoreside.moos \
                            DEPLOY_ALL=true \
            MOOS_MANUAL_OVERRIDE_ALL=false \
                            RETURN_ALL=false \
                    STATION_KEEP_ALL=false \
                        DEMUSTER_ALL=false \
                        DEMUSTERING=true \
                    DEMUSTER_ASSIGN=true \
                    DEMUSTER_CONFIG=type_circle \
                        DEMUSTER_BEGIN=true \
            &> /dev/null &

        DONE="false"

        #State monitoring machine, sustaining checks before bringing the mission down
        while [ "${DONE}" = "false" ] ; do 
            sleep 3
            t_now=$(date +%s)
            mission_duration=$((t_now-mission_start))
            #1) Has the process time for this session ran over? This would imply a hanging process or application, if it has been exceeded, we cut it
            if [ $mission_duration -gt $PROCESS_TIME ] ; then
                echo "   Process TimeOut" 
                DONE="true"
                # Get the most recent mission log folder and move it to the failed attempts folder
                # mv $(ls -td ./logs_simulation/$BATCH_NAME/*/ | head -n 1) ./logs_simulation/$BATCH_NAME/failed_attempts/
                mission_failed=true
                break
            fi

            #2) Have we received a QUIT_MISSION queue
            if uQueryDB targ_shoreside.moos           \
                --condition="DEMUSTER_COMPLETE == true" --wait=3 >& /dev/null ; then 
                DONE="true"
                break
                #3) Have we been running over the allotted expected mission time?
                # TODO: Ask Ray why include this and the one above?
            elif uQueryDB targ_shoreside.moos         \
                    --condition="DB_UPTIME >= $MISSION_TIME" --wait=2 >& /dev/null ; then 
                echo "   Mission TimeOut" 
                DONE="true"
                mission_failed=true
                break
            fi
        done
        # sleep 2

        #Make sure every single process is brought down
        kill $pid_l
        nuke_moos $pid_l &
        # pkill -f '^MOOSDB'
        # pkill uMAC
        # pKill pShare
        # pKill uProcessWatch
        # pKill pLogger
        # pKill pRealm
        # pKill uPokeDB

        sleep 2
        ktm &> /dev/null
        sleep 2
        # kill -9 $pid_l

        # Get the folder of the last mission
        newest_mission=logs_simulation/$BATCH_NAME/$mission_name
        ./log_strip.sh $newest_mission

        sleep 2

        # If the mission failed, move the mission to the failed attempts folder
        if [ "$mission_failed" = true ]; then
            failed_attempts=$((failed_attempts+1))
            mv $newest_mission ./logs_simulation/$BATCH_NAME/failed_attempts/
            
            if [ $failed_attempts -le 3 ]; then
                idx=$((idx-1))
                trial_idx=$((trial_idx-1))
                continue
            else
                echo "Mission failed 3 times, skipping..."
                failed_attempts=0
            fi
        else 
            failed_attempts=0
        fi
                
    done
done



# Need to wait for the last post processor to finish

# wait $!


# BUG: Anythign after this point is not executed
echo "All missions complete!"


# Post processing on the batch folder
./batch_dubin_post_process.sh logs_simulation/$BATCH_NAME

# Calculate dubin_metrics on the batch folder
./dubin_metrics.sh logs_simulation/$BATCH_NAME

# afplay /path/to/sound/file.mp3