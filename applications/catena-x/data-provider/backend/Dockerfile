FROM maven:3.8-jdk-11 AS build
COPY . /backend
RUN mvn -f backend/pom.xml clean install

FROM adoptopenjdk:11
COPY --from=build /backend /backend
EXPOSE 8082
ENTRYPOINT ["java","-jar","/backend/target/backend-0.0.1-SNAPSHOT.jar"]
