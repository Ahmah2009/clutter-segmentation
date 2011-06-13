all: main.pdf

main.pdf: main.tex
	if test -f ~/Mendeley/Bachelorarbeit.bib ; then cp ~/Mendeley/Bachelorarbeit.bib bibliography/literature.bib ; fi
	pdflatex main.tex
	bibtex main.aux
	makeglossaries main
	makeindex main.tex
	pdflatex main.tex
	pdflatex main.tex
	pdflatex main.tex

main.tex: main.Rnw chapters/introduction.tex chapters/theory.tex chapters/implementation.tex chapters/evaluation.tex chapters/conclusion.tex
	R CMD Sweave main.Rnw

chapters/introduction.tex: chapters/introduction.Rnw
	cd chapters && R CMD Sweave introduction.Rnw

chapters/theory.tex: chapters/theory.Rnw
	cd chapters && R CMD Sweave theory.Rnw

chapters/implementation.tex: chapters/implementation.Rnw
	cd chapters && R CMD Sweave implementation.Rnw

chapters/evaluation.tex: chapters/evaluation.Rnw
	cd chapters && R CMD Sweave evaluation.Rnw

chapters/conclusion.tex: chapters/conclusion.Rnw
	cd chapters && R CMD Sweave conclusion.Rnw

.PHONY: install-deps
install-deps:
	sudo aptitude install r-recommended r-cran-dbi
	wget http://cran.r-project.org/src/contrib/RSQLite_0.9-4.tar.gz
	sudo R CMD INSTALL RSQLite_0.9-4.tar.gz
	rm RSQLite_0.9-4.tar.gz

.PHONY: clean 
clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux *.bbl *.blg *.lof *.lot *.xdy

