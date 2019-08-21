PACKAGE de.monticore.lang.monticar.sol

IMPORT de.monticore.lang.monticar.sol.SubFile

DOCKERFILE Component

// FROM
FROM "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/utilities/Sol/build:latest"

//CMD
CMD ["SomeCoolCommand", "SomeCoolParameter", "SomeOtherCoolParameter"]
CMD ["SomeCoolParameter", "SomeOtherCoolParameter"]
CMD "SomeCoolCommand That Nobody Understands"

// ADD
ADD --chown=4 "source1" "source2" "destination"
ADD --chown=4:5 ["source1", "source2", "destination"]
ADD "source1" "source2" "destination"
ADD ["source1", "source2", "destination"]

// COPY
COPY --chown=4 "source1" "source2" "destination"
COPY --chown=4:5 ["source1", "source2", "destination"]
COPY "source1" "source2" "destination"
COPY ["source1", "source2", "destination"]

// ENTRYPOINT
ENTRYPOINT ["SomeCoolCommand", "SomeCoolParameter", "SomeOtherCoolParameter"]
ENTRYPOINT "SomeCoolCommand SomeCoolParameter"

// USER
USER 4:5
USER "peter":"hans"

// ONBUILD
ONBUILD ADD ["source1", "source2", "destination"]

// STOPSIGNAL
STOPSIGNAL 0
STOPSIGNAL "SIGTERM"

// HEALTHCHECK
HEALTHCHECK --interval="4s" CMD "SomeCommand"
HEALTHCHECK --timeout="4s" CMD "SomeCommand"
HEALTHCHECK --start-period="4s" CMD "SomeCommand"
HEALTHCHECK --retries=4 CMD "SomeCommand"
HEALTHCHECK NONE