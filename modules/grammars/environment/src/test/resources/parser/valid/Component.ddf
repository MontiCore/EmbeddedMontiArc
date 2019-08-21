PACKAGE de.monticore.lang.monticar.sol

IMPORT de.monticore.lang.monticar.sol.Component

COMPONENT DOCKERFILE Base

// LABEL
LABEL "com.example.vendor" = "ACME Incorporated"

// INSTALL
INSTALL "openjdk", "openjdk2"
INSTALL "openjdk"

// RUN
RUN ["/bin/bash", "-c", "echo hello"]
RUN "/bin/bash -c 'source $HOME /.bashrc; echo $HOME"

// ENV
ENV "myName" = "John Doe"
ENV "myName" "John Doe"

// EXPOSE
EXPOSE 3000

// VOLUME
VOLUME ["/data"]
VOLUME "/var/log" "/var/db"

// WORKDIR
WORKDIR "/path/to/workdir"

// ARG
ARG "user1"
ARG "buildno"="1"

// SHELL
SHELL ["/bin/bash", "-c"]