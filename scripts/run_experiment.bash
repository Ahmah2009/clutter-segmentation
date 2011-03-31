#!/bin/bash

# TODO: rename
# TODO: csv output
# TODO: make stddev input parameter
# TODO: make truth file an input parameter
if [ "$1" = "--help" ] ; then
    cat <<HELP
Usage: run_experiment.bash [-p] [-r] [-n] [-u]

Runs pose randomization test.
-p prepare training bases
-r randomize poses
-n generate statistics with noisy pose estimates
-u generate statistics with undistorted pose estimates
HELP
    exit
fi

if [ ! "$CLUTSEG_PATH" ] ; then
    echo "ERROR: Environment variable CLUTSEG_PATH is not defined."
    exit
fi


flag_prepare=
flag_randomize=
flag_stats_noisy=
flag_stats_undistorted=

while getopts 'prnu' OPTION
do
  case $OPTION in
  p)    flag_prepare=1
        ;;
  r)    flag_randomize=1
        ;;
  n)    flag_stats_noisy=1
        ;;
  u)    flag_stats_undistorted=1
        ;;
  esac
done
shift $(($OPTIND - 1))

# TODO: get parameters
# TODO: write results to some file
# TODO: run against already prepared test setup
stddev_t=0.01
stddev_r=0.03
truthfile=$CLUTSEG_PATH/test/test-truth.txt

pushd $CLUTSEG_PATH/poserandomization > /dev/null

    # Prepare base
    if [ "$flag_prepare" ]
    then
        rm undistorted -rf
        rm noisy -rf
        mkdir undistorted
        mkdir noisy

        echo "Dump bag files to first training base ..."
        rosrun tod_training dump_all.py bags undistorted 

        cp bags/fiducial.yml undistorted/
        cp bags/features.config.yaml undistorted/
        cp bags/config.yaml undistorted/
        cp bags/config.txt undistorted/

        # Run pose estimation on first training base
        # Run masking on first training base
        pushd undistorted
            for subject in $(cat config.txt); do
                echo "estimating pose for $subject"
                rosrun tod_training pose_estimator -d $subject --verbose=false
                echo "creating mask for $subject"
                rosrun tod_training masker -d $subject -M 1 --verbose=false
            done
        popd 

        # Copy over everything to second training base
        cp -r undistorted/* noisy/ --verbose

        # Run detection on first training base
        pushd undistorted
        for subject in $(cat config.txt); do
            echo "extracting features for $subject in undistorted"
            rosrun tod_training detector -d $subject --verbose=true
            rosrun tod_training f3d_creator -d $subject --verbose=true
        done
        popd
    fi

    if [ "$flag_randomize" ]
    then
        for subject in $(cat undistorted/config.txt); do
            cp -r --verbose undistorted/$subject/*.pose.yaml noisy/$subject/
            rosrun semantic3d_training poserandomizer noisy/$subject $stddev_t $stddev_r 1 0
        done

        # TODO: Add Noise parameter model to experiment records 

        # Run detection on second training base
        pushd noisy 
        for subject in $(cat config.txt); do
            echo "extracting features for $subject in noisy"
            rosrun tod_training detector -d $subject --verbose=true
            rosrun tod_training f3d_creator -d $subject --verbose=true
        done
        popd
    fi

    semantic3d_training=`rospack find semantic3d_training`

    # TODO: remove duplication
    if [ "$flag_stats_undistorted" ]
    then
        res=result-undistorted_$(date +%Y-%m-%d_%H-%M-%S)
     
        tod_training_pkg=`rospack find tod_training`
        tod_detecting_pkg=`rospack find tod_detecting`
        echo "Experiment running on "`date` >> $res
        echo "--------------------------------------------------" >> $res
        echo "--------------------------------------------------" >> $res
        echo "stddev_t = $stddev_t, stddev_r = $stddev_t" >> $res
        echo "--------------------------------------------------" >> $res
        svn info $tod_training_pkg >> $res 
        echo "--------------------------------------------------" >> $res
        svn info $tod_detecting_pkg >> $res 
        echo "--------------------------------------------------" >> $res
        echo "detection mode: $mode" >> $res
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "config.yaml" >> $res
        echo "--------------------------------------------------" >> $res
        cat undistorted/config.yaml >> $res 
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "" >> $res
        echo "features.config.yaml" >> $res
        echo "--------------------------------------------------" >> $res
        cat undistorted/features.config.yaml >> $res 
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "" >> $res


        # Do recognition using first training base
        # Save recognition results to experiment records
        # $semantic3d_training/scripts/gen_training_base_truth.py --base undistorted undistorted-truth.txt
        echo "Running statistics using undistorted training base" > $res

        $semantic3d_training/scripts/recognizer_stats.py --base undistorted -t $truthfile -f undistorted/config.yaml --verbose -m 1 > $res
        echo "stddev_t = $stddev_t, stddev_r = $stddev_t" >> $res
        echo "" >> $res
    fi

    if [ "$flag_stats_noisy" ]
    then
        res=result-noisy_$(date +%Y-%m-%d_%H-%M-%S)
        
        tod_training_pkg=`rospack find tod_training`
        tod_detecting_pkg=`rospack find tod_detecting`
        echo "Experiment running on "`date` >> $res
        echo "--------------------------------------------------" >> $res
        echo "--------------------------------------------------" >> $res
        echo "stddev_t = $stddev_t, stddev_r = $stddev_t" >> $res
        echo "--------------------------------------------------" >> $res
        svn info $tod_training_pkg >> $res 
        echo "--------------------------------------------------" >> $res
        svn info $tod_detecting_pkg >> $res 
        echo "--------------------------------------------------" >> $res
        echo "detection mode: $mode" >> $res
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "config.yaml" >> $res
        echo "--------------------------------------------------" >> $res
        cat noisy/config.yaml >> $res 
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "" >> $res
        echo "features.config.yaml" >> $res
        echo "--------------------------------------------------" >> $res
        cat noisy/features.config.yaml >> $res 
        echo "--------------------------------------------------" >> $res
        echo "" >> $res
        echo "" >> $res


        # Do recognition using second training base
        # Save recognition results to experiment records
        # $semantic3d_training/scripts/gen_training_base_truth.py --base noisy noisy-truth.txt
        echo "Running statistics using noisy training base" > $res
        $semantic3d_training/scripts/recognizer_stats.py --base noisy -t $truthfile -f noisy/config.yaml --verbose -m 1 > $res
        echo "stddev_t = $stddev_t, stddev_r = $stddev_t" >> $res
        echo "" >> $res
    fi

popd > /dev/null

