#!/usr/bin/env bash

function usage() {
    cat <<USAGE
Usage: result-html [post-trigger-cmd]

Generates a HTML overview about experiment results. Requires
CLUTSEG_EXPERIMENT_DB, CLUTSEG_RESULT_DIR and CLUTSEG_ARTIFACT_DIR to be set in
the environment. Requires R and sqlite3 command line interface installed.
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
cat >> $out <<EOF
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

p {
    width: 500px;
}
</style>
<body>
EOF

function table() {
    echo "<h1>$1</h1>" >> $out
    echo "<table>" >> $out
    sqlite3 -header -html $CLUTSEG_EXPERIMENT_DB "$2"  >> $out
    echo "</table>" >> $out
}

set -f

echo "<h1>Target</h1>" >> $out
result-best-succ-rate
echo "<img src='best_succ_rate.png' />" >> $out
cat >> $out <<EOF
<p>The ultimate target in this experiment to achieve a very high success rate.
The bar indicates the best success rate achieved so far in any experiment that
has been carried out so far. 100% success is achieved if on every test scene,
one object is correctly labeled and located correctly up to a certain margin of
error for rotation and translation.</p>
EOF

table "Scores" "select * from view_experiment_scores"
cat >> $out <<EOF
<p>If many objects have been successfully located, yet with comparatively large
errors, then <tt>locate_sipc</tt> is smaller than <tt>succ_rate</tt>. In case,
many objects have been correctly classified but not correctly located, then
<tt>succ_rate</tt> will be smaller than <tt>locate_sipc</tt>.</p>
EOF

table "Locate SIPC Scores" "select * from view_experiment_locate_sipc"
cat >> $out <<EOF
<p>See <a href='http://code.in.tum.de/indefero/index.php//p/clutter-segmentation/source/tree/master/clutseg/include/clutseg/sipc.h'>
sipc.h</a> for a description.</p>
EOF

table "Detect SIPC Scores" "select * from view_experiment_detect_sipc"
cat >> $out <<EOF
<p>See <a href='http://code.in.tum.de/indefero/index.php//p/clutter-segmentation/source/tree/master/clutseg/include/clutseg/sipc.h'>
sipc.h</a> for a description.</p>
EOF

table "Receiver Operating Characteristics" "select * from view_experiment_detect_roc"
result-roc
echo "<img src='detect_roc.png' />" >> $out

table "Errors" "select * from view_experiment_error"

table "Runtimes" "select * from view_experiment_runtime"

table "Notes" "select * from view_experiment_note"

echo "<h1>Test data</h1>" >> $out
echo "<a href='image_all.jpg'><img src='image_all.jpg' width='500' /></a>" >> $out

cat >> $out <<EOF
<p>This is the set of 21 test images, for which we have ground truth.</p>
EOF


echo "</body></html>" >> $out

