# ---------------------------------------------------------------------
#   LaTeX Makefile
#   XeLaTeX + minted + biber + glossaries + index
#   Uses: out/ for build, dist/ for final PDF
# ---------------------------------------------------------------------

MAIN = main
OUTDIR = out
DISTDIR = dist

LATEXMK = latexmk
LATEXMK_OPTS = -lualatex -shell-escape -outdir=$(OUTDIR) \
               -interaction=nonstopmode -synctex=1

TEX_SRCS = $(shell find . -name "*.tex")
BIB_SRCS = $(shell find . -name "*.bib")

# ---------------------------------------------------------------------
# Master target
# ---------------------------------------------------------------------
all: dirs prebuild glossary index finalbuild copy

dirs:
	mkdir -p $(OUTDIR)
	mkdir -p $(DISTDIR)

# ---------------------------------------------------------------------
# Step 1: First latexmk run (produces .glo, .idx, etc.)
# ---------------------------------------------------------------------
prebuild: $(OUTDIR)/$(MAIN).prebuild

$(OUTDIR)/$(MAIN).prebuild: $(TEX_SRCS) $(BIB_SRCS)
	$(LATEXMK) $(LATEXMK_OPTS) $(MAIN).tex
	touch $(OUTDIR)/$(MAIN).prebuild

# ---------------------------------------------------------------------
# Step 2: Glossary generation
# ---------------------------------------------------------------------
glossary:
	if [ -f "$(OUTDIR)/$(MAIN).glo" ]; then \
		makeglossaries -d $(OUTDIR) $(MAIN); \
	fi

# ---------------------------------------------------------------------
# Step 3: Index generation
# ---------------------------------------------------------------------
index:
	if [ -f "$(OUTDIR)/$(MAIN).idx" ]; then \
		makeindex $(OUTDIR)/$(MAIN).idx; \
	fi

# ---------------------------------------------------------------------
# Step 4: Final latexmk run to include glossary + index
# ---------------------------------------------------------------------
finalbuild:
	$(LATEXMK) $(LATEXMK_OPTS) $(MAIN).tex

# ---------------------------------------------------------------------
# Step 5: Copy final PDF to dist/
# ---------------------------------------------------------------------
copy:
	cp $(OUTDIR)/$(MAIN).pdf $(DISTDIR)/$(MAIN).pdf

# ---------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------
clean:
	$(LATEXMK) -outdir=$(OUTDIR) -c

veryclean:
	$(LATEXMK) -outdir=$(OUTDIR) -C
	rm -f $(DISTDIR)/$(MAIN).pdf
	rm -f $(OUTDIR)/$(MAIN).prebuild

.PHONY: all dirs prebuild glossary index finalbuild copy clean veryclean
