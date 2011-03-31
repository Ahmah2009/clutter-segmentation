#!/usr/bin/env bash
#
# Reformats a C source file.
#

indent --indent-level4 \
       --no-tabs \
       --case-indentation4 \
       --brace-indent0 \
       --dont-break-procedure-type \
       --no-space-after-function-call-names \
       --braces-on-if-line \
       --cuddle-do-while \
       --cuddle-else $1

