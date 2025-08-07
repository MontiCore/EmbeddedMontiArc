FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/utilities/gdltools/gdltools:latest

RUN mkdir /environment
COPY . /environment
RUN bash -c "cd /environment && dos2unix *.sh"
RUN bash -c "cd /environment && ./make.sh"

ENTRYPOINT bash -c "cd /environment && ./train.sh"
