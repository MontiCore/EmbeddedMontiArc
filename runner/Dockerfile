#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#


FROM maven:3.6-jdk-8

WORKDIR /app

RUN apt-get update &&\
    apt-get install -y openjfx

COPY ./ ./

RUN mvn clean install -s settings.xml -DskipTests
