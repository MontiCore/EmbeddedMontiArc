<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="side" type="java.lang.String" -->
<#-- @ftlvariable name="tasks" type="java.util.List<de.monticore.lang.monticar.sol.grammars.ide._symboltable.TaskSymbol>" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "configuration", "side")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = configuration.getName()>
<#assign fullname = configuration.getFullName()>
<#if side == "Frontend">
<#assign tasks = configuration.getFrontendTaskSymbols()>
<#else>
<#assign tasks = configuration.getBackendTaskSymbols()>
</#if>
<#assign options = name + "Options">
import { injectable } from "inversify";
import { Common${side}ConfigurationRunner } from "@embeddedmontiarc/sol-runtime-configurations/lib/<#if side == "Frontend">browser<#else>node</#if>";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";

<#assign qualifiedPath = printer.printQualifiedPath(configuration)>
import { ${options} } from "../../../common/configurations/${qualifiedPath}/protocol";

@injectable()
export class ${name}${side}Runner<#if hasHandwrittenPeer>TOP</#if> extends Common${side}ConfigurationRunner<${options}> {
    public constructor() {
        super("${fullname}");
    }

    public async run(uuid: string, taskName: string, options: ${options}, context: OptionsContext, token: CancellationToken): Promise<void> {
        switch (taskName) {
            <#list tasks as task>
            <#assign taskName = task.getName()>
            <#assign methodSuffix = printer.printMethodNameSuffix(task)>
            case "${taskName}": return this.run${methodSuffix}(uuid, options, context, token);
            </#list>
        }
    }

    <#list tasks as task>
    <#assign methodSuffix = printer.printMethodNameSuffix(task)>
    protected run${methodSuffix}(uuid: string, options: ${options}, context: OptionsContext, token: CancellationToken): Promise<void> {
        return Promise.resolve(undefined);
    }
    </#list>
}
