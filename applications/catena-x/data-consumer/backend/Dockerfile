FROM maven:3.8-jdk-11 AS build
COPY . /backend
RUN mvn --settings /backend/settings.xml -f backend/pom.xml clean install

FROM adoptopenjdk:11
COPY --from=build /backend /backend
EXPOSE 8080
ENTRYPOINT ["java","-jar","/backend/infrastructure/target/infrastructure-0.0.1-SNAPSHOT.jar"]
