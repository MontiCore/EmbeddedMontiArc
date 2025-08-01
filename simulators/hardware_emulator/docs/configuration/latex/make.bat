@REM
@REM (c) https://github.com/MontiCore/monticore
@REM



@echo off
set GENDIR=target\
set MAINNAME=emulator_configuration
set MAINTEX=%MAINNAME%.tex
set TARGET=%GENDIR%%MAINNAME%.pdf


mkdir %GENDIR%
@echo ------------initial pdf-latex run
pdflatex -output-directory=%GENDIR% %MAINTEX%

@echo ------------Make index
IF EXIST "%GENDIR%%MAINNAME%.idx" makeindex %GENDIR%%MAINNAME%.idx
@echo ------------2nd pdf-latex run
pdflatex -output-directory=%GENDIR% %MAINTEX%
@echo ------------3rd pdf-latex run
pdflatex -output-directory=%GENDIR% %MAINTEX%
