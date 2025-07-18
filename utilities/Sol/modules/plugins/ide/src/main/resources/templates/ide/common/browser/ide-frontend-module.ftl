<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign name = ide.getName()>
<#assign cinclusions = ide.getConfigurationTypeInclusionSymbols()>
<#assign cexclusions = ide.getConfigurationTypeExclusionSymbols()>
<#assign minclusions = ide.getModuleTypeInclusionSymbols()>
<#assign mexclusions = ide.getModuleTypeExclusionSymbols()>
<#assign namelc = name?lower_case>
import { ContainerModule } from "inversify";
import { ModuleTypeContribution } from "@embeddedmontiarc/sol-runtime-modules/lib/browser";
import { ConfigurationTypeContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/browser";
import { ConfigurationRunnerContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";

import { ${name}ModuleTypeContribution } from "./${namelc}-module-type-contribution";
import { ${name}ConfigurationTypeContribution } from "./${namelc}-configuration-type-contribution";
import { ${name}FrontendRunnerContribution } from "./${namelc}-frontend-runner-contribution";
import { ${name}FrontendValidatorContribution } from "./${namelc}-frontend-validator-contribution";

<#list cinclusions as cinclusion>
<#assign cname = cinclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(cinclusion)>
import {
    ${cname}FrontendRunner as ${qualifiedClass}FrontendRunner
} from "./configurations/${qualifiedPath}/frontend-runner";
import {
    ${cname}FrontendValidator as ${qualifiedClass}FrontendValidator
} from "./configurations/${qualifiedPath}/frontend-validator";
</#list>

<#list minclusions as minclusion>
<#assign mname = minclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(minclusion)>
import {
    ${mname}FrontendValidator as ${qualifiedClass}FrontendValidator
} from "./modules/${qualifiedPath}/frontend-validator";
</#list>

export default new ContainerModule(bind => {
    <#list cinclusions as cinclusion>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
    bind(${qualifiedClass}FrontendRunner).toSelf().inSingletonScope();
    bind(${qualifiedClass}FrontendValidator).toSelf().inSingletonScope();
    </#list>

    <#list minclusions as minclusion>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
    bind(${qualifiedClass}FrontendValidator).toSelf().inSingletonScope();
    </#list>

    bind(ModuleTypeContribution).to(${name}ModuleTypeContribution).inSingletonScope();
    bind(ConfigurationTypeContribution).to(${name}ConfigurationTypeContribution).inSingletonScope();
    bind(ConfigurationRunnerContribution).to(${name}FrontendRunnerContribution).inSingletonScope();
    bind(ValidatorContribution).to(${name}FrontendValidatorContribution).inSingletonScope();
});
