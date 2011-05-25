main: main.Rnw
	R CMD Sweave main.Rnw
	cd chapters && R CMD Sweave results.Rnw
	pdflatex main.tex
	makeglos.pl main
	makeindex main.tex
	pdflatex main.tex

clean:	
	rm -f *.glo *.glg *.log *.out *.toc *.ist *.ind *.ilg *.gls *.aux

