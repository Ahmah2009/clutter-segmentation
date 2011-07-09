#!/usr/bin/env bash

source ~/.env

function script_init() {
    if has_opt -h || has_opt --help ; then
        usage
        exit
    fi
}

function print_usage() {
    name=$1
    shift
    echo -e "usage: \033[1m${name}\033[0m $*"
}

function has_opt() {
    arg=$1

    for (( i=0; i<${#OPTS[@]}; i++)); do
        a=${OPTS[${i}]}
        if [ "$a" = "$arg" ] || [ "$a" = "$arg" ] ; then
            return 0
        fi
    done
    return 1 
}

function has_no_opt() {
    if has_opt $1 ; then
        return 1
    else
        return 0
    fi
}

function expect_arg() {
    if [ "${ARGS[$1]}" = "" ] ; then
        echo "Not enough arguments."
        echo
        usage
        exit
    fi
}

function get_arg() {
    echo "${ARGS[$1]}"
}

 # generic helpers

function assert_base() {
    if ! [ -f fiducial.yml ] ; then
        cat <<ERROR
No fiducial.yml in training/testing base. Exiting for safety reasons. Note that
the existence of this file is just a flag that shows this a training or testing
directory.
ERROR
        exit
    fi
}

CMDARGS=$*
ARGS=
OPTS=

declare -a ARGS
declare -a OPTS
argi=0
opti=0
# sort for options and arguments
for a in $* ; do
    if [[ "$a" == --* ]] || [[ "$a" == -? ]] ; then
        OPTS[$opti]=$a
        opti=$(($opti + 1))
    else
        ARGS[$argi]=$a
        argi=$(($argi + 1))
    fi
done

script_init

# echo "OPTS=${OPTS[@]}"
# echo "ARGS=${ARGS[@]}"

