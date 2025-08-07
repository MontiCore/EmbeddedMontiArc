<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tool" type="de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="package" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "package", "tool")}//tslint:disable
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign isVirtual = tool.isVirtual()>
<#assign name = tool.getName()>
<#assign qualifiedFolder = tool.getFullName()?lower_case>
<#assign directory = package.getDirectory("artifacts").get()>
import { injectable } from "inversify";
import { Common<#if isVirtual>Virtual</#if>Tool } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";

<#if !isVirtual>
import * as path from "path";
</#if>

@injectable()
export class ${name}<#if isVirtual>Virtual</#if>Tool<#if hasHandwrittenPeer>TOP</#if> extends Common<#if isVirtual>Virtual</#if>Tool {
    public constructor() {
        <#if isVirtual>
        super("${tool.getCommand().get()}");
        <#else>
        <#assign path = tool.getPath().get()>
        const artifactsDirectory = path.resolve(__dirname, "../../..", "${directory}");
        const basename = path.basename("${path}");
        const absolutePath = path.resolve(artifactsDirectory, "${qualifiedFolder}", basename);

        super(absolutePath, "${tool.getPrefix().orElse("")}", "${tool.getSuffix().orElse("")}");
        </#if>
    }
}
