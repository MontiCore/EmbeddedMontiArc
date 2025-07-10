PACKAGE DDFCollectorImpl

IMPORT DDFCollectorImpl.SecondFile

DOCKERFILE FirstFile

FROM "SomeImage:SomeVersion"

INSTALL "cmake", "python"

ARG "abc" = "def"

EXPOSE 300

SHELL ["/bin/bash", "-c"]

VOLUME ["/some/volume"]

ONBUILD CMD "Do Something"

WORKDIR "/to/work/directory"