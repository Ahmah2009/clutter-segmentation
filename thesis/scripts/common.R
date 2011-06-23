#!/usr/bin/env Rscript
library(RSQLite)
# FIXME: connect to correct database
con = dbConnect(dbDriver("SQLite"), dbname = paste(Sys.getenv("CLUTSEG_PATH"), "/clutter-segmentation/thesis/media/experiments.sqlite3", sep=""))

