main: main.Rnw
	cp ~/Mendeley/Bachelorarbeit.bib bibliography/literature.bib
	R CMD Sweave main.Rnw
	cd chapters && R CMD Sweave results.Rnw
	bibtex main
	pdflatex main.tex
	bibtex main
	makeglos.pl main
	makeindex main.tex
	pdflatex main.tex

clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux *.bbl *.blg

