main: main.Rnw
	if test -f ~/Mendeley/Bachelorarbeit.bib ; then cp ~/Mendeley/Bachelorarbeit.bib bibliography/literature.bib ; fi
	R CMD Sweave main.Rnw
	#cd chapters && R CMD Sweave bonanza.Rnw
	cd chapters && R CMD Sweave introduction.Rnw
	cd chapters && R CMD Sweave theory.Rnw
	cd chapters && R CMD Sweave implementation.Rnw
	cd chapters && R CMD Sweave evaluation.Rnw
	cd chapters && R CMD Sweave conclusion.Rnw
	pdflatex main.tex
	bibtex main
	pdflatex main.tex
	bibtex main
	makeglos.pl main
	makeindex main.tex
	pdflatex main.tex

.PHONY: install-deps
install-deps:
	sudo aptitude install r-recommended r-cran-dbi
	wget http://cran.r-project.org/src/contrib/RSQLite_0.9-4.tar.gz
	sudo R CMD INSTALL RSQLite_0.9-4.tar.gz
	rm RSQLite_0.9-4.tar.gz

.PHONY: clean 
clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux *.bbl *.blg *.lof *.lot

