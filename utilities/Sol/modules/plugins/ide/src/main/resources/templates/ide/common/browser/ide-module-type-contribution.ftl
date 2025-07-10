<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign name = ide.getName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign inclusions = ide.getModuleTypeInclusionSymbols()>
<#assign exclusions = ide.getModuleTypeExclusionSymbols()>
import { injectable } from "inversify";
import { ModuleTypeContribution, ModuleTypeRegistry } from "@embeddedmontiarc/sol-runtime-modules/lib/browser";

@injectable()
export class ${name}ModuleTypeContribution<#if hasHandwrittenPeer>TOP</#if> implements ModuleTypeContribution {
    public registerModuleTypes(registry: ModuleTypeRegistry): void {
        <#list inclusions as inclusion>
        <#assign id = inclusion.getFullName()>
        <#assign label = inclusion.getLabel()>
        <#assign iconClass = inclusion.getIcon()>
        <#assign category = inclusion.getCategory()>
        registry.registerModuleType({
            id: "${id}",
            label: "${label}",
            <#if iconClass.isPresent()>iconClass: "${iconClass.get()}",</#if>
            <#if category.isPresent()>category: "${category.get()}"</#if>
        });
        </#list>

        <#list exclusions as exclusion>
        <#assign id = exclusion.getFullName()>
        registry.unregisterModuleType("${id}");
        </#list>
    }
}
