<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="option" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="suffix" type="java.lang.String" -->
${tc.signature("option", "suffix")}
<#assign delegator = glex.getGlobalVar("option.delegator")>
<#assign name = option.getName()>
<#assign subOptions = delegator.getOptionSymbols(option)>
${name} = ${name} || {};

return {
    <#list subOptions as subOption>
    <#assign subOptionName = subOption.getName()>
    ${subOptionName}: await this.validate${suffix}(${name}.${subOptionName}, options)<#if subOption?has_next>,</#if>
    </#list>
};
