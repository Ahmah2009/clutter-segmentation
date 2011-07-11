all: main.pdf
WARNING=grep "Warning" -
WARNING=cat
main.pdf: main.tex
	#if test -f ~/Mendeley/Bachelorarbeit.bib ; then cp ~/Mendeley/Bachelorarbeit.bib bibliography/literature.bib ; fi
	pdflatex main.tex | ${WARNING}
	bibtex main.aux | ${WARNING}
	makeglossaries main | ${WARNING}
	makeindex main.tex | ${WARNING}
	pdflatex main.tex | ${WARNING}
	pdflatex main.tex | ${WARNING}

main.tex: main.Rnw chapters/*.tex bibliography/literature.bib components/*.tex
	Rscript scripts/PngSweave.R main.Rnw

chapters/introduction.tex: chapters/introduction.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R introduction.Rnw

chapters/theory.tex: chapters/theory.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R theory.Rnw

chapters/implementation.tex: chapters/implementation.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R implementation.Rnw

chapters/evaluation.tex: chapters/evaluation.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R evaluation.Rnw

chapters/conclusion.tex: chapters/conclusion.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R conclusion.Rnw

chapters/appendix.tex: chapters/appendix.Rnw
	cd chapters && Rscript ../scripts/PngSweave.R appendix.Rnw

.PHONY: install-deps
install-deps:
	sudo aptitude install r-recommended r-cran-dbi
	wget http://cran.r-project.org/src/contrib/RSQLite_0.9-4.tar.gz
	sudo R CMD INSTALL RSQLite_0.9-4.tar.gz
	rm RSQLite_0.9-4.tar.gz
	sudo aptitude install texlive-latex3 texlive-humanities

.PHONY: clean 
clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux *.bbl *.blg *.lof *.lot *.xdy *.bib

