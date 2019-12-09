<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="serializer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer.OptionsSerializer" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign serializer = glex.getGlobalVar("option.serializer")>
<#assign name = ide.getName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign inclusions = ide.getConfigurationTypeInclusionSymbols(false)>
<#assign exclusions = ide.getConfigurationTypeExclusionSymbols(false)>
import { injectable } from "inversify";

import {
    ConfigurationTypeContribution, ConfigurationTypeRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/browser";

@injectable()
export class ${name}ConfigurationTypeContribution<#if hasHandwrittenPeer>TOP</#if> implements ConfigurationTypeContribution {
    public registerConfigurationTypes(registry: ConfigurationTypeRegistry): void {
        /*
         * Inclusions
         */
        <#list inclusions as inclusion>
        <#assign id = inclusion.getFullName()>
        <#assign label = inclusion.getLabel()>
        <#assign iconClass = inclusion.getIcon()>
        <#assign category = inclusion.getCategory()>
        <#assign options = inclusion.getOptions()>
        registry.registerConfigurationType({
            id: "${id}",
            label: "${label}",
            <#if iconClass.isPresent()>iconClass: "${iconClass.get()}",</#if>
            <#if category.isPresent()>category: "${category.get()}",</#if>
            options: ${serializer.serialize(inclusion)}
        });
        </#list>

        /*
         * Exclusions
         */
        <#list exclusions as exclusion>
        <#assign id = exclusion.getFullName()>
        registry.unregisterConfigurationType("${id}");
        </#list>
    }
}
