#!/bin/sh
set -e

pdflatex main
biber main
pdflatex main
pdflatex main
