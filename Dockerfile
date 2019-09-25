#
# This Dockerfile is used as image description for the container used to test external functionality.
# In production user rights should be more restrictive.
#
FROM registry.git.rwth-aachen.de/monticore/embeddedmontiarc/utilities/sol/build:latest

# Parts taken from https://github.com/theia-ide/theia-apps/blob/master/theia-full-docker/Dockerfile
ENV DEBIAN_FRONTEND noninteractive

ENV LC_ALL C.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en

RUN apt-get update && \
    apt-get -y install curl && \
    curl https://winswitch.org/gpg.asc | apt-key add - && \
    echo "deb http://winswitch.org/ bionic main" > /etc/apt/sources.list.d/xpra.list && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get -y install git websockify xpra xvfb python-numpy firefox xterm && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/project && \
    mkdir -p /home/sol

WORKDIR /home/sol

ENV USER root

#
# One day Docker will be able to handle proper wildcards in COPY, but that day is not today...
#
COPY yarn.lock .
COPY package.json .

COPY modules/tests/ide/package.json modules/tests/ide/package.json
COPY modules/tests/language/language-client/package.json modules/tests/language/language-client/package.json

COPY packages/development/tsc/package.json packages/development/tsc/package.json
COPY packages/development/cli/package.json packages/development/cli/package.json

COPY packages/external/core/package.json packages/external/core/package.json
COPY packages/external/docker/package.json packages/external/docker/package.json
COPY packages/external/preparation/package.json packages/external/preparation/package.json
COPY packages/external/workspace/package.json packages/external/workspace/package.json
COPY packages/external/gui-process/package.json packages/external/gui-process/package.json
COPY packages/external/monaco/package.json packages/external/monaco/package.json

COPY packages/runtime/components/package.json packages/runtime/components/package.json
COPY packages/runtime/modules/package.json packages/runtime/modules/package.json
COPY packages/runtime/templates/package.json packages/runtime/templates/package.json

COPY packages/internal/se-logo/package.json packages/internal/se-logo/package.json

COPY packages/tests/templates/package.json packages/tests/templates/package.json

RUN rm -rf ./ycache && \
    NODE_OPTIONS="--max_old_space_size=4096" YARN_CACHE_FOLDER="./ycache" yarn install --ignore-scripts --frozen-lockfile

COPY . .

RUN NODE_OPTIONS="--max_old_space_size=4096" YARN_CACHE_FOLDER="./ycache" yarn install --force

EXPOSE 3000

ENV SHELL /bin/bash
ENV DISPLAY :10

WORKDIR /home/sol/modules/tests/ide

ENTRYPOINT ["yarn", "start", "/home/project", "--hostname=0.0.0.0"]
