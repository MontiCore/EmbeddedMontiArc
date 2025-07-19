// LABEL
LABEL com.exaple.vendor = "ACME Incorporated"

// INSTALL
INSTALL openjdk=0.0.1-SNAPSHOT
INSTALL "openjdk"

// RUN
RUN ["/bin/bash", "-c", "echo hello"]
RUN "/bin/bash -c 'source $HOME /.bashrc; echo $HOME"

// ENV
ENV "myName" = "John Doe"
ENV "myName" John Doe

// EXPOSE
EXPOSE NoNumber

// VOLUME
VOLUME ["/data"]
VOLUME "/var/log" "/var/db"

// WORKDIR
WORKDIR "/path/to/workdir"

// ARG
ARG "user1"
ARG "buildno"=1

// SHELL
SHELL "/bin/bash", "-c"
