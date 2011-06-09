#!/usr/bin/env bash
#

trap "exit" INT TERM EXIT

source $CLUTSEG_PATH/experiment_setup.bash

migrate_to_labelset=$(rospack find clutseg)/bin/migrate_to_labelset

pushd $CLUTSEG_RESULT_DIR > /dev/null
    for r in $(find -maxdepth 1 -iname "?????" -type d | sort) ; do
        r=${r:2}
        echo -n "Fixing stored choices in $r ... "
        find $r -iname "*.detect_choices.yaml.gz" | xargs -n 1 $migrate_to_labelset 2>&1 >> migrate.log
        find $r -iname "*.locate_choice.yaml.gz" | xargs -n 1 $migrate_to_labelset 2>&1 >> migrate.log
        echo "[OK]"
    done
popd > /dev/null
