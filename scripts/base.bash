#!/usr/bin/env bash

function script_init() {
    for a in $* ; do
        if [ "$a" = "-h" ] || [ "$a" = "--help" ] ; then
            usage
            exit
        fi
    done
}

script_init $*

