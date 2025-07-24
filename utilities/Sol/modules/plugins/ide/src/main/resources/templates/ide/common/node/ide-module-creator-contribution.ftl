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
<#assign inclusions = ide.getModuleTypeInclusionSymbols()>
<#assign exclusions = ide.getModuleTypeExclusionSymbols()>
import { injectable } from "inversify";
import { ModuleCreatorRegistry, ModuleCreatorContribution } from "@embeddedmontiarc/sol-runtime-modules/lib/node";

<#list inclusions as inclusion>
<#assign iname = inclusion.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
<#assign qualifiedPath = printer.printQualifiedPath(inclusion)>
import { ${iname}ModuleCreator as ${qualifiedClass}ModuleCreator } from "./modules/${qualifiedPath}/creator";
</#list>

@injectable()
export class ${name}ModuleCreatorContribution implements ModuleCreatorContribution {
    public registerModuleCreators(registry: ModuleCreatorRegistry): void {
        <#list inclusions as inclusion>
        <#assign qualifiedClass = printer.printQualifiedClassPrefix(inclusion)>
        registry.registerModuleCreator(${qualifiedClass}ModuleCreator);
        </#list>

        <#list exclusions as exclusion>
        <#assign id = exclusion.getFullName()>
        registry.unregisterModuleCreator("${id}");
        </#list>
    }
}
