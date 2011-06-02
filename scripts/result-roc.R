#!/usr/bin/env Rscript
library(RSQLite)
con = dbConnect(dbDriver("SQLite"), dbname = Sys.getenv("CLUTSEG_EXPERIMENT_DB"))
f = dbGetQuery(con, "select detect_fp_rate, detect_tp_rate from view_experiment_detect_roc")
attach(f)
png(paste(Sys.getenv("CLUTSEG_ARTIFACT_DIR"), "/detect_roc.png", sep=""))
plot(detect_fp_rate, detect_tp_rate, xlim=c(0, 1), ylim=c(0, 1), main="Detection", xlab="False Positive Rate", ylab = "True Positive Rate")
abline(0, 1, col="GRAY")
