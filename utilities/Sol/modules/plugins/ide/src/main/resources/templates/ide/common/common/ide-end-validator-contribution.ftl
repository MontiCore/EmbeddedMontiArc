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
<#assign name = ide.getName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign cinclusions = ide.getConfigurationTypeInclusionSymbols(false)>
<#assign cexclusions = ide.getConfigurationTypeExclusionSymbols(false)>
<#assign minclusions = ide.getModuleTypeInclusionSymbols()>
<#assign mexclusions = ide.getModuleTypeExclusionSymbols()>
import { injectable } from "inversify";
import { ValidatorContribution, ValidatorRegistry } from "@embeddedmontiarc/sol-runtime-options/lib/common";

<#list cinclusions as cinclusion>
<#assign cname = cinclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(cinclusion)>
import {
    ${cname}${side}Validator as ${qualifiedClass}${side}Validator
} from "./configurations/${qualifiedPath}/${side?lower_case}-validator";
</#list>

<#list minclusions as minclusion>
<#assign mname = minclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(minclusion)>
import {
    ${mname}${side}Validator as ${qualifiedClass}${side}Validator
} from "./modules/${qualifiedPath}/${side?lower_case}-validator";
</#list>

@injectable()
export class ${name}${side}ValidatorContribution<#if hasHandwrittenPeer>TOP</#if> implements ValidatorContribution {
    public registerValidators(registry: ValidatorRegistry): void {
        <#list cinclusions as cinclusion>
        <#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
        registry.registerValidator(${qualifiedClass}${side}Validator);
        </#list>

        <#list minclusions as minclusion>
        <#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
        registry.registerValidator(${qualifiedClass}${side}Validator);
        </#list>

        <#list cexclusions as cexclusion>
        <#assign id = cexclusion.getFullName()>
        registry.unregisterValidator("${id}");
        </#list>

        <#list mexclusions as mexclusion>
        <#assign id = mexclusion.getFullName()>
        registry.unregisterValidator("${id}");
        </#list>
    }
}
