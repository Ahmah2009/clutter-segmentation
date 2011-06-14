#!/usr/bin/env bash

if [ "$1" = "" ] ; then
    cat <<EOF
Usage: install <directory>

Installs the clutter-segmentation tool to the specified directory. If the
directory does not exist, it will be created. If it already exists, it must be
empty. The tool requires several dependencies and their installation requires
root privileges. The installation script has been tested on Ubuntu Maverick,
but should also run on similar systems.
EOF
    exit
fi

d=$1

if [ -d $d ] ; then
    if [ "$(ls -A $d)" ]; then
        echo "ERROR: $d is not empty."
        exit
    fi
else
    mkdir -p $d
fi

# $1 requirement 
# $2 command 
# $3 description
function step() {
    printf "%-55s " "$3"
    if ! $1 ; then
        printf "%-12s " "please wait"
        $2 >> /dev/null
    else
        printf "%-12s " "skipping"
    fi
    $1 && echo "[SUCCESS]" || echo "[FAILURE]"
}

function available() {
    [ "$(which $1)" != "" ]
}

ALWAYS=0
RSQLITE=RSQLite_0.9-4.tar.gz
RCRAN=http://cran.r-project.org/src/contrib

pushd $d >> /dev/null
    step "[ -d clutter-segmentation ]"  "git clone --quiet indefero@code.in.tum.de:clutter-segmentation.git"        "Cloning repository clutter-segmentation.git"
    step "available rosinstall"         "sudo easy_install rosinstall"                                              "Installing rosinstall tool" 
    step $ALWAYS                        "rosinstall . clutter-segmentation/clutter-segmentation.rosinstall"         "Checking out dependencies via rosinstall"
    step $ALWAYS                        "source setup.bash"                                                         "Sourcing environment via setup.bash"
    step $ALWAYS                        "rosdep install clutseg"                                                    "Installing dependencies via rosdep"
    step $ALWAYS                        "export CLUTSEG_PATH=$(pwd)"                                                "Exporting CLUTSEG_PATH"
    step $ALWAYS                        "export PATH=$CLUTSEG_PATH/clutter-segmentation/scripts/script-bin:$PATH"   "Amending PATH"
    step $ALWAYS                        "mods-link"                                                                 "Applying patches and modifications"
    step $ALWAYS                        "rosmake --rosdep-install clutseg"                                          "Building clutseg using rosmake"
    step $ALWAYS                        "wget $RCRAN/$RSQLITE && R CMD INSTALL $RSQLITE && rm $RSQLITE"             "Installing RSQLite"
popd >> /dev/null

