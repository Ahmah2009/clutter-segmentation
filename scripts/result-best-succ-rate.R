#!/usr/bin/env Rscript
library(RSQLite)
con = dbConnect(dbDriver("SQLite"), dbname = paste(Sys.getenv("CLUTSEG_PATH"), "/clutter-segmentation/clutseg/build/paramsel.sqlite3", sep=""))
f = dbGetQuery(con, "select succ_rate from response order by succ_rate DESC limit 1")
attach(f)
png(paste(Sys.getenv("CLUTSEG_ARTIFACT_DIR"), "/best_succ_rate.png", sep=""))
barplot(matrix(c(succ_rate, 1-succ_rate), 2, 1), names.arg=c("Best Success Rate"), horiz=TRUE, col=c("GREEN", "RED"))
