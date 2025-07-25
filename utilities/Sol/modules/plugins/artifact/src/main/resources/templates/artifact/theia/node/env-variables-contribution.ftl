<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="name" type="java.lang.String" -->
<#-- @ftlvariable name="artifacts" type="java.util.List<de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol>" -->
<#-- @ftlvariable name="tools" type="java.util.List<de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol>" -->
<#-- @ftlvariable name="package" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "name", "package", "artifacts", "tools")}//tslint:disable
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign directory = package.getDirectory("artifacts").get()>
import { injectable } from "inversify";
import { EnvVariablesContribution, EnvVariablesRegistry } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";

import * as path from "path";

@injectable()
export class ${name}EnvVariablesContribution<#if hasHandwrittenPeer>TOP</#if> implements EnvVariablesContribution {
    public registerEnvVariables(registry: EnvVariablesRegistry): void {
        <#list artifacts as artifact>
        <#assign alias = artifact.getAlias()>
        <#if alias.isPresent()>
        <#assign path = artifact.getPath().get()>
        registry.registerEnvVariable({
            name: "${alias.get()}",
            value: this.resolveArtifact("${path}")
        });
        </#if>
        </#list>

        <#list tools as tool>
        <#assign alias = tool.getAlias()>
        <#if alias.isPresent() && !tool.isVirtual()>
        <#assign qualifiedFolder = tool.getFullName()?lower_case>
        <#assign path = tool.getPath().get()>
        registry.registerEnvVariable({
            name: "${alias.get()}",
            value: this.resolveArtifact("${qualifiedFolder}", "${path}")
        });
        </#if>
        </#list>
    }

    protected resolveArtifact(qualifiedFolder: string, source: string): string {
        const artifactsDirectory = path.resolve(__dirname, "../..", "${directory}");
        const basename = path.basename(source);

        return path.resolve(artifactsDirectory, qualifiedFolder, basename);
    }
}
