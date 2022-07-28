FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/reinforcement_learning/cartpole/rl-fix-new2:latest

RUN apt update
RUN apt install software-properties-common -y
RUN add-apt-repository ppa:openjdk-r/ppa -y
RUN apt-add-repository ppa:swi-prolog/stable -y
RUN apt update
RUN apt install openjdk-11-jdk maven swi-prolog dos2unix -y

RUN mkdir /environment
COPY . /environment
RUN bash -c "cd /environment && dos2unix *.sh"
RUN bash -c "cd /environment && ./make.sh"

EXPOSE 8080
ENTRYPOINT bash -c "cd /environment && ./train.sh"
