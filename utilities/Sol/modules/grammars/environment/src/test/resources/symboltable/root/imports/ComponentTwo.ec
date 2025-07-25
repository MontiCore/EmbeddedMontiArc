PACKAGE root.imports

COMPONENT DOCKERFILE ComponentTwo

LABEL "com.example.vendor" = "ACME Incorporated"

INSTALL "openjdk"

RUN ["/bin/bash", "-c", "echo hello"]

ENV "myName" "John Doe"

EXPOSE 3000

WORKDIR "/path/to/workdir"

ARG "buildno"="1"

SHELL ["/bin/bash", "-c"]