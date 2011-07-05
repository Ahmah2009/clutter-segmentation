if (file.exists("scripts/Thibaut_Jombart_Sweave.R")) {
    source("scripts/Thibaut_Jombart_Sweave.R")
} else if (file.exists("../scripts/Thibaut_Jombart_Sweave.R")) {
    source("../scripts/Thibaut_Jombart_Sweave.R")
} else if (file.exists("Thibaut_Jombart_Sweave.R")) {
    source("Thibaut_Jombart_Sweave.R")
}
Sweave(commandArgs(TRUE)[1])
