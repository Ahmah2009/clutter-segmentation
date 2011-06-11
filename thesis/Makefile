main: main.Rnw
	cp ~/Mendeley/Bachelorarbeit.bib bibliography/literature.bib
	R CMD Sweave main.Rnw
	cd chapters && R CMD Sweave bonanza.Rnw
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

clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux *.bbl *.blg *.lof *.lot

