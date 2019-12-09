<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="artifacts" type="java.util.List<de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol>" -->
<#-- @ftlvariable name="tools" type="java.util.List<de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol>" -->
<#-- @ftlvariable name="name" type="java.lang.String" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "name", "artifacts", "tools")}//tslint:disable
import { ContainerModule } from "inversify";
import { bindToolFactory } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";
import { EnvVariablesContribution } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";

import { ${name}EnvVariablesContribution } from "./${name?lower_case}-env-variables-contribution";

<#list tools as tool>
<#assign isVirtual = tool.isVirtual()>
<#assign tname = tool.getName()>
<#assign qualifiedFolder = tool.getFullName()?lower_case>
import { ${tname}<#if isVirtual>Virtual</#if>Tool } from "./${qualifiedFolder}/tool";
import { ${tname}<#if isVirtual>Virtual</#if>ToolFactory } from "./${qualifiedFolder}/tool-factory";
</#list>

<#list artifacts as artifact>
<#assign aname = artifact.getName()>
<#assign qualifiedFolder = artifact.getFullName()?lower_case>
import { ${aname}Artifact } from "./${qualifiedFolder}/artifact";
</#list>

export default new ContainerModule(bind => {
    <#list tools as tool>
    <#assign tname = tool.getName()>
    <#assign isVirtual = tool.isVirtual()>
    bindToolFactory(bind, ${tname}<#if isVirtual>Virtual</#if>ToolFactory, ${tname}<#if isVirtual>Virtual</#if>Tool);
    </#list>

    <#list artifacts as artifact>
    <#assign aname = artifact.getName()>
    bind(${aname}Artifact).toSelf().inSingletonScope();
    </#list>

    bind(EnvVariablesContribution).to(${name}EnvVariablesContribution).inSingletonScope();
});
