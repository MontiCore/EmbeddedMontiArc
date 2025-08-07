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
<#assign name = ide.getName()>
<#assign cinclusions = ide.getConfigurationTypeInclusionSymbols()>
<#assign minclusions = ide.getModuleTypeInclusionSymbols()>
<#assign namelc = name?lower_case>
import { ContainerModule } from "inversify";
import { ModuleCreatorContribution } from "@embeddedmontiarc/sol-runtime-modules/lib/node";
import { ConfigurationRunnerContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { ConfigurationCoordinatorContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";

import { ${name}ModuleCreatorContribution } from "./${namelc}-module-creator-contribution";

import { ${name}BackendRunnerContribution } from "./${namelc}-backend-runner-contribution";

import { ${name}BackendValidatorContribution } from "./${namelc}-backend-validator-contribution";

import { ${name}CoordinatorContribution } from "./${namelc}-coordinator-contribution";

<#list cinclusions as cinclusion>
<#assign cname = cinclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(cinclusion)>
import {
    ${cname}BackendRunner as ${qualifiedClass}BackendRunner
} from "./configurations/${qualifiedPath}/backend-runner";
import {
    ${cname}BackendValidator as ${qualifiedClass}BackendValidator
} from "./configurations/${qualifiedPath}/backend-validator";
import {
    ${cname}Coordinator as ${qualifiedClass}Coordinator
} from "./configurations/${qualifiedPath}/coordinator";
</#list>

<#list minclusions as minclusion>
<#assign mname = minclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(minclusion)>
import {
    ${mname}BackendValidator as ${qualifiedClass}BackendValidator
} from "./modules/${qualifiedPath}/backend-validator";
import {
    ${mname}ModuleCreator as ${qualifiedClass}ModuleCreator
} from "./modules/${qualifiedPath}/creator";
</#list>

export default new ContainerModule(bind => {
    <#list cinclusions as cinclusion>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(cinclusion)>
    bind(${qualifiedClass}BackendRunner).toSelf().inSingletonScope();
    bind(${qualifiedClass}BackendValidator).toSelf().inSingletonScope();
    bind(${qualifiedClass}Coordinator).toSelf().inSingletonScope();
    </#list>

    <#list minclusions as minclusion>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(minclusion)>
    bind(${qualifiedClass}BackendValidator).toSelf().inSingletonScope();
    bind(${qualifiedClass}ModuleCreator).toSelf().inSingletonScope();
    </#list>

    bind(ModuleCreatorContribution).to(${name}ModuleCreatorContribution).inSingletonScope();
    bind(ConfigurationRunnerContribution).to(${name}BackendRunnerContribution).inSingletonScope();
    bind(ValidatorContribution).to(${name}BackendValidatorContribution).inSingletonScope();
    bind(ConfigurationCoordinatorContribution).to(${name}CoordinatorContribution).inSingletonScope();
});
