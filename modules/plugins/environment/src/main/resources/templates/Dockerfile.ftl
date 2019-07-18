<#-- @ftlvariable name="instructions" type="java.lang.String" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.environment.plugin.generate.configuration.EnvironmentGenerateConfiguration" -->
<#compress>
${tc.signature("instructions")}
<#assign configuration = glex.getGlobalVar("configuration")>
FROM ${configuration.getBaseImage()}
</#compress>

${instructions}
#
# IDE
#
RUN adduser --disabled-password --gecos '' theia && \
    adduser theia sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers;

RUN chmod g+rw /home && \
    mkdir -p /home/project && \
    chown -R theia:theia /home/theia && \
    chown -R theia:theia /home/project;

RUN apt-get update && apt-get install -y python build-essential

USER theia

WORKDIR /home/theia

ADD package.json ./package.json

RUN yarn --cache-folder ./ycache && \
    rm -rf ./ycache && \
    NODE_OPTIONS="--max_old_space_size=4096" yarn theia build

EXPOSE 3000

ENV SHELL /bin/bash

ENTRYPOINT ["yarn", "theia", "start", "/home/project", "--hostname=0.0.0.0"]