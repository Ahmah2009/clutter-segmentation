#!/usr/bin/env bash

ARGS=$*

function script_init() {
    if has_arg -h ; then
        echo "help requested"
        usage
        exit
    fi
}

function has_arg() {
    arg=$1
    for a in $ARGS ; do
        if [ "$a" = "$arg" ] || [ "$a" = "$arg" ] ; then
            return 0
        fi
    done
    return 1 
}

function has_no_arg() {
    if has_arg $1 ; then
        return 1
    else
        return 0
    fi
}

function expect_arg() {
    if [ "$1" = "" ] ; then
        usage
        exit
    fi
}

script_init $ARGS

