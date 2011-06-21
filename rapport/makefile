all: rapport slides

rapport:
	xelatex rapport
	bibtex rapport
	xelatex rapport
	xelatex rapport

slides:
	xelatex slides

clean:
	rm *.log
	rm *.aux
	rm *.blg
	rm *.bbl