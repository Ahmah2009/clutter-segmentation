source(paste(Sys.getenv("CLUTSEG_PATH"), "clutter-segmentation/thesis/scripts/Thibaut_Jombart_Sweave.R", sep="/"))
Sweave(commandArgs(TRUE)[1])
