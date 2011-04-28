#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: base-pose <base> [--no-fix] [--leave-fix]

Performs pose estimation on images with fiducial markers. If flag '--no-fix' is
given no pre-processed alternative images will provided and existing
alternative image for pose estimations will be removed. Flag --leave-fix will
not create alternative images but leave existing untouched. An alternative
image especially designed as a fallback for tod_training pose_estimator has the
name <image>.pre.pose.png. 
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

expect_arg 0
base=$(get_arg 0)

pushd $CLUTSEG_PATH/$base > /dev/null
    assert_training_base
    for d in *; do
        assert_training_base
        if [ -d $d ]; then
            subj=$(basename $d)
            echo "Estimating poses for $subj"
            echo "--------------------------------------------------------"
            pushd $subj >/dev/null
                if has_no_opt --no-fix && has_no_opt --leave-fix; then
                    for img in image_?????.png ; do
                        echo "Generating alternative training image for $subj/$img ..."
                        convert -monochrome $img $img.pre.pose.png
                    done
                elif ! [ has_opt --leave-fix ] ; then
                    rm *.pre.pose.png
                fi
            popd > /dev/null
            echo "Running tod_training pose estimator..."
            # only run with one job, seems to be some racing condition
            # but I did not attempt to find the bug
            rosrun tod_training pose_estimator -d $subj -j1
       fi
    done
popd > /dev/null
