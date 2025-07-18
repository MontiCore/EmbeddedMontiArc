<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="side" type="java.lang.String" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide", "side")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = ide.getName()>
<#assign inclusions = ide.getConfigurationTypeInclusionSymbols()>
<#assign exclusions = ide.getConfigurationTypeExclusionSymbols()>
import { injectable } from "inversify";

import {
    ConfigurationRunnerContribution, ConfigurationRunnerRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/common";

<#list inclusions as inclusion>
<#assign iname = inclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(inclusion)>
import {
    ${iname}${side}Runner as ${qualifiedClass}${side}Runner
} from "./configurations/${qualifiedPath}/${side?lower_case}-runner";
</#list>

@injectable()
export class ${name}${side}RunnerContribution<#if hasHandwrittenPeer>TOP</#if> implements ConfigurationRunnerContribution {
    public registerRunners(registry: ConfigurationRunnerRegistry): void {
        <#list inclusions as inclusion>
        <#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
        registry.registerRunner(${qualifiedClass}${side}Runner);
        </#list>

        <#list exclusions as exclusion>
        <#assign fullname = exclusion.getFullName()>
        registry.unregisterRunner(${fullname});
        </#list>
    }
}
