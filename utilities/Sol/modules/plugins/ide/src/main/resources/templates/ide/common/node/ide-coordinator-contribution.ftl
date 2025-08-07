<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = ide.getName()>
<#assign inclusions = ide.getConfigurationTypeInclusionSymbols()>
<#assign exclusions = ide.getConfigurationTypeExclusionSymbols()>
import { injectable } from "inversify";

import {
    ConfigurationCoordinatorContribution, ConfigurationCoordinatorRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/node";

<#list inclusions as inclusion>
<#if !inclusion.isComponent()>
<#assign iname = inclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(inclusion)>
import {
    ${iname}Coordinator as ${qualifiedClass}Coordinator
} from "./configurations/${qualifiedPath}/coordinator";
</#if>
</#list>

@injectable()
export class ${name}CoordinatorContribution<#if hasHandwrittenPeer>TOP</#if> implements ConfigurationCoordinatorContribution {
    public registerCoordinators(registry: ConfigurationCoordinatorRegistry): void {
        <#list inclusions as inclusion>
        <#if !inclusion.isComponent()>
        <#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
        registry.registerCoordinator(${qualifiedClass}Coordinator);
        </#if>
        </#list>

        <#list exclusions as exclusion>
        <#if !exclusion.isComponent()>
        <#assign fullname = exclusion.getFullName()>
        registry.unregisterCoordinator("${fullname}");
        </#if>
        </#list>
    }
}
