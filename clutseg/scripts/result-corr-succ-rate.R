#!/usr/bin/env Rscript
library(RSQLite)
# correlate-succ-rate <title> <x> <y> <view> <filename> [<xmin>,<xmax> [<ymin>,<ymax>]]
# export CLUTSEG_EXPERIMENT_DB="/media/29FAF067618013AC/experiments.sqlite3" &&
# ./correlate-succ-rate.R "Detector Receiver Operating Characteristics" value refine_sipc "view_experiment_scores" /home/julius/Temp/correlate-succ-rate.png 0,1 0,1
# TODO: rename!

args = commandArgs(TRUE)
title = args[1]
x = args[2]
y = args[3]
view = args[4]
filename = args[5]

f = dbGetQuery(
    dbConnect(dbDriver("SQLite"), dbname = Sys.getenv("CLUTSEG_EXPERIMENT_DB")),
    paste("select ", paste(x, y, "succ_rate", sep=", "), " from ", view))

png(filename)

# True optional arguments not supported, if necessary use CRAN optparse
if (length(args) > 5 && args[6] != "") {
    xl = as.numeric(unlist(strsplit(args[6], ",")))
} else {
    xl = range(f[,1])
}
if (length(args) > 6 && args[7] != "") {
    yl = as.numeric(unlist(strsplit(args[7], ",")))
} else {
    yl = range(f[,2])
}

PALETTE = c("darkred", "red", "orange", "green", "darkgreen")
plot(f[,2] ~ f[,1], type="p", main=title,
    xlim=xl, ylim=yl, xlab=x, ylab=y,
    col=PALETTE[1. + floor(length(PALETTE) * f[,3])])

