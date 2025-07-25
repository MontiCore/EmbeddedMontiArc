<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="language" type="de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="side" type="java.lang.String" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "language", "side")}// tslint:disable
<#assign name = language.getName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign declarations = language.getLocalDeclarationSymbols()>
<#assign undeclarations = language.getLocalUndeclarationSymbols()>
import { injectable } from "inversify";
import { ValidatorContribution, ValidatorRegistry } from "@embeddedmontiarc/sol-runtime-options/lib/common";

<#list declarations as declaration>
<#assign dname = declaration.getName()>
import { ${dname?cap_first}${side}Validator } from "./validators/${dname?lower_case}-${side?lower_case}-validator";
</#list>

@injectable()
export class ${name}${side}ValidatorContribution<#if hasHandwrittenPeer>TOP</#if> implements ValidatorContribution {
    public registerValidators(registry: ValidatorRegistry): void {
        <#list declarations as declaration>
        <#assign dname = declaration.getName()>
        registry.registerValidator(${dname?cap_first}${side}Validator);
        </#list>

        <#list undeclarations as undeclaration>
        <#assign id = undeclaration.getFullName()>
        registry.unregisterValidator("${id}");
        </#list>
    }
}
