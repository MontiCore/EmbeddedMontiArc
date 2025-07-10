<#-- (c) https://github.com/MontiCore/monticore -->
${tc.signature("ide")}
<#assign name = ide.getName()>
<#assign package = ide.getPackageName()>
<#assign environments = ide.getEnvironmentSymbols()>
PACKAGE ${package}

<#list environments as environment>
IMPORT ${environment.getFullName()}
</#list>

DOCKERFILE ${name}

FROM "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/utilities/sol/base:latest"

// Parts taken from https://github.com/theia-ide/theia-apps/blob/master/theia-full-docker/Dockerfile
ENV "DEBIAN_FRONTEND" = "noninteractive"

ENV "LC_ALL" = "C.UTF-8"
ENV "LANG" = "en_US.UTF-8"
ENV "LANGUAGE" = "en_US:en"

INSTALL "curl"

RUN "curl https://winswitch.org/gpg.asc | apt-key add -"
RUN "echo \"deb http://winswitch.org/ bionic main\" > /etc/apt/sources.list.d/xpra.list"

INSTALL "software-properties-common"

RUN "add-apt-repository universe"

INSTALL "git", "websockify", "xpra", "xvfb", "python-numpy", "xterm"

RUN "mkdir -p /home/workspace"
RUN "mkdir -p /home/${name?lower_case}"
RUN "mkdir -p /home/plugins"

WORKDIR "/home/${name?lower_case}"

ENV "USER" = "root"

RUN "rm -rf ./ycache"

COPY "." "."

RUN "NODE_OPTIONS=\"--max_old_space_size=4096\" YARN_CACHE_FOLDER=\"./ycache\" yarn install"

EXPOSE 3000

ENV "SHELL" = "/bin/bash"
ENV "DISPLAY" = ":10"
ENV "THEIA_DEFAULT_PLUGINS" = "local-dir:/home/plugins"

ENTRYPOINT ["yarn", "start", "/home/workspace", "--hostname=0.0.0.0"]
