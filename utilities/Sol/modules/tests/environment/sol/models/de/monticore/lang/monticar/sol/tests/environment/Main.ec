PACKAGE de.monticore.lang.monticar.sol.tests.environment

IMPORT de.monticore.lang.monticar.sol.tests.environment.FirstEnvironment
IMPORT de.monticore.lang.monticar.sol.tests.environment.SecondEnvironment

DOCKERFILE Main

FROM "ALPHA"

INSTALL "some"
INSTALL "other"
INSTALL "awesome"
INSTALL "packages"

LABEL "somelabel" = "somevalue"
LABEL "somelabel2" = "somevalue"

RUN "something to run"

VOLUME ["/show/me/them/other/volumes"]

WORKDIR "/stay/out"

RUN "something forbidden"

SHELL ["/a/red/shell", "-c"]

ARG "somearg2" = "some value"

ENV "SOME_OTHER_ENV" = "some value"

EXPOSE 5000

STOPSIGNAL "SIGTERM"

ONBUILD RUN "SomeCommand"

HEALTHCHECK --retries=4 CMD "DoSomething"
HEALTHCHECK NONE

USER "theia":"something"
USER 0:10

ENTRYPOINT ["top", "-b"]

CMD "echo \"This is a test.\" | wc -"

COPY --from="x" "test" "relativeDir"
COPY --chown=20:20 "test" "/absoluteDir/"

ADD --chown=20:20 "test" "/absoluteDir/"