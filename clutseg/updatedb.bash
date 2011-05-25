#!/usr/bin/env bash
cd $CLUTSEG_PATH
source setup.bash
roscd clutseg
rm data/test.sqlite3 
rm data/paramsel.sqlite3 
cat schema/*.sql | sqlite3 data/test.sqlite3
cat schema/*.sql | sqlite3 data/paramsel.sqlite3
cat view/*.sql | sqlite3 data/test.sqlite3
cat view/*.sql | sqlite3 data/paramsel.sqlite3
cat data/test.sql | sqlite3 data/test.sqlite3 
cat data/paramsel.sql | sqlite3 data/paramsel.sqlite3 
cat schema/*.sql | sqlite3 $CLUTSEG_PATH/experiments.sqlite3
