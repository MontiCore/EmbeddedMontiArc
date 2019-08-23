<!-- (c) https://github.com/MontiCore/monticore -->
# WebServerForDemonstrator

The web-server based on the Jetty web-server. The main goal is to process requests from the demonstartor front-end part. The server does the following:
1. Extract incoming model's files.
2. Copy the files to a folder for the further processing (../models/uniqueName/controllerName).
3. Run the compilation process using the compile.sh script (../script.sh).
4. Archive the compiled data.
5. Send the compiled controller back to the front-end part.

To build the server, Maven is used and builds it with dependencies:
```
mvn clean compile assembly:single
```

To run the server should be executed the following:
```
java -jar web-server-demonstrator.jar PORT
```
