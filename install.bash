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
        echo -e -n "\033[1m"
        c=""
        while [ "$c" != "y" ] && [ "$c" != "n" ] ; do
            echo "Directory $d is not empty. Continue [y/n]? "
            read c
        done
        tput sgr0
        if [ "$c" != "y" ] ; then
            exit 1
        fi
    fi
else
    mkdir -p $d
fi

function print_status() {
    col=""
    if [ "$3" = "SUCCESS" ] ; then 
        col="\E[32m"
    fi
    printf "\033[1m$col%-55s %-12s [%-7s]\033[0m\n" "$1" "$2" "$3"
    tput sgr0
}

# $1 requirement 
# $2 command 
# $3 description
function step() {
    if $1 ; then
        print_status "$3" "please wait" "PENDING"
        echo "Executing <$2> ..."
        $2 # >> /dev/null
        s=$?
        m=$([ "$s" = "0" ] && echo "SUCCESS" || echo "FAILURE")
        print_status "$3" "finished" "$m"
        if [ "$s" != "0" ] ; then
            echo -e '\033[1m\E[31mFAILURE.'
            tput sgr0
            exit 1
        fi
    else
        print_status "$3" "skipped" "SUCCESS"
    fi
}

function not_a_dir() {
    if [ -d "$1" ] ; then
        return 1
    else
        return 0
    fi 
}

function not_available() {
    [ "$(which $1)" = "" ]
}

function always() {
    return 0 
}

RSQLITE='RSQLite_0.9-4.tar.gz'
RCRAN='http://cran.r-project.org/src/contrib'
ROS_STACKS='ros-diamondback-object-manipulation ros-diamondback-pr2-object-manipulation ros-diamondback-pr2-common-actions ros-diamondback-pr2-cockpit ros-diamondback-tabletop-object-perception'

pushd $d >> /dev/null
    step "not_a_dir clutter-segmentation"   "git clone indefero@code.in.tum.de:clutter-segmentation.git"               "Cloning repository clutter-segmentation.git"
    step "not_available rosinstall"         "sudo easy_install rosinstall"                                             "Installing rosinstall tool" 
    step "always"                           "rosinstall . clutter-segmentation/clutter-segmentation.rosinstall"        "Checking out dependencies via rosinstall"
    step "always"                           "source setup.bash"                                                        "Sourcing environment via setup.bash"
    step "always"                           "rosdep install clutseg"                                                   "Installing dependencies via rosdep"
    step "always"                           "export CLUTSEG_PATH=$(pwd)"                                               "Exporting CLUTSEG_PATH"
    step "always"                           "rosrun clutseg mods-link"                                                 "Applying patches and modifications"
    step "always"                           "rosdep install clutseg"                                                   "Installing clutseg system dependencies"
    step "always"                           "sudo apt-get install $ROS_STACKS"                                         "Installing additional stacks for ROS diamondback"
    step "always"                           "rosmake clutseg"                                                          "Building clutseg using rosmake"
    step "always"                           "roscd clutseg"                                                            "Changing to clutseg path"
    step "always"                           "make tests"                                                               "Compiling clutseg tests"
    step "always"                           "wget $RCRAN/$RSQLITE"                                                     "Downloading RSQLite Archive"
    step "always"                           "sudo R CMD INSTALL $RSQLITE"                                              "Installing RSQLite"
    step "always"                           "rm $RSQLITE"                                                              "Removing RSQLite Archive"
    step "always"                           "bin/utest --gtest_filter=test_extractor.orb_extract_features"             "Running simple ORB test"
    step "always"                           "bin/utest --gtest_filter=test_clutseg.recog_using_tar"                    "Running recognition test"
popd >> /dev/null

