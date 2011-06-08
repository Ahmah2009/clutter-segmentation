#!/usr/bin/env bash
#

trap "exit" INT TERM EXIT

source $CLUTSEG_PATH/experiment_setup.bash

pose_to_posert=$(rospack find clutseg)/bin/pose_to_posert

pushd $CLUTSEG_RESULT_DIR > /dev/null
    for r in $(find -maxdepth 1 -iname "?????" -type d | sort) ; do
        r=${r:2}
        echo -n "Fixing stored choices in $r ... "
        find $r -iname "*.detect_choices.yaml.gz" | xargs -l $pose_to_posert 
        find $r -iname "*.locate_choice.yaml.gz" | xargs -l $pose_to_posert
        echo "[OK]"
    done
popd > /dev/null
