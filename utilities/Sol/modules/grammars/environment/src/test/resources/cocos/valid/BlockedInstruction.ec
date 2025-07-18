PACKAGE valid

COMPONENT DOCKERFILE BlockedInstruction

LABEL "com.example.vendor" = "ACME Incorporated"

INSTALL "openjdk", "openjdk2"
INSTALL "openjdk"

RUN ["/bin/bash", "-c", "echo hello"]
RUN "/bin/bash -c 'source $HOME /.bashrc; echo $HOME"

ENV "myName" = "John Doe"
ENV "myName" "John Doe"

EXPOSE 3000

VOLUME ["/data"]
VOLUME "/var/log" "/var/db"

WORKDIR "/path/to/workdir"

ARG "user1"
ARG "buildno"="1"

SHELL ["/bin/bash", "-c"]