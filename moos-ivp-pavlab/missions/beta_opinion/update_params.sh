#!/bin/bash
#---------------------------------------------------------------
#   Script: update_params.sh
#  Mission: beta_opinion
#   Author: Tyler Paine
#   LastEd: Dec 2024
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }


# URL of the file to download
url="https://oceanai.mit.edu/monte/results/tpaine/GCID_learn/baseline/alpha_job/results.csv"

# Name of the downloaded file
filename="results_file.csv"

# Download the file
wget -O "$filename" "$url"

# Check if download was successful
if [ $? -eq 0 ]; then
    echo "File downloaded successfully!"

    # Call the Python script
    python process_mission.py "$filename"

    echo "                                  "
    echo "----------------------------------"
    echo "Updated parameter file params.txt:"
    cat params.txt

    echo "                                  "
    echo "----------------------------------"
    
    echo "Commiting this parameter set for  "
    echo "other machines to use             "
    svn commit -m "automated commit for parameter updates"
    

else
    echo "File download failed."
fi
