#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: result-html

Generates a HTML overview about experiment results. Requires
CLUTSEG_EXPERIMENT_DB, CLUTSEG_RESULT_DIR and CLUTSEG_ARTIFACT_DIR to be
set in the environment.
USAGE
}

source $CLUTSEG_PATH/clutter-segmentation/scripts/base.bash $*

if [ "$CLUTSEG_EXPERIMENT_DB" = "" ] ; then
    usage
    echo "CLUTSEG_EXPERIMENT_DB not set."
    exit 1
fi

if [ "$CLUTSEG_RESULT_DIR" = "" ] ; then
    usage
    echo "CLUTSEG_RESULT_DIR not set."
    exit 1
fi

if [ "$CLUTSEG_ARTIFACT_DIR" = "" ] ; then
    usage
    echo "CLUTSEG_ARTIFACT_DIR not set."
    exit 1
fi

out=$CLUTSEG_ARTIFACT_DIR/results.html

echo > $out
tee $out <<START
<html>
<style>

body {
    margin: 20px 60px;
}

* {
    font-family: monospace;
}

table {
    border-spacing: 0;
    border-collapse: collapse;
}

th, td {
    border: 1px solid black;
    padding: 5px;
    margin: 0;
    font-size: 10px;
}

th {
    background-color: rgb(255, 203, 153);
}
</style>
<body>
START

function table() {
    echo $1
    echo $2
    echo "<h1>$1</h1>" >> $out
    echo "<table>" >> $out
    sqlite3 -header -html $CLUTSEG_EXPERIMENT_DB "$2"  >> $out
    echo "</table>" >> $out
}

set -f
table "Receiver Operating Characteristics" "select * from view_experiment_detect_roc"
result-roc
echo "<img src='detect_roc.png' />" >> $out
table "Scores" "select * from view_experiment_scores"
table "Errors" "select * from view_experiment_error"
result-best-succ-rate
echo "<img src='best_succ_rate.png' />" >> $out

echo "</body></html>" >> $out
