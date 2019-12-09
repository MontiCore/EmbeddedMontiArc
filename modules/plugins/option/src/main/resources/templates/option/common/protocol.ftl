<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.OptionPrinter" -->
<#-- @ftlvariable name="symbol" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
${tc.signature("template", "symbol")}
<#assign delegator = glex.getGlobalVar("option.delegator")>
<#assign printer = glex.getGlobalVar("option.printer")>
<#assign name = symbol.getName()>
<#assign options = delegator.getOptionSymbols(symbol)>
/*
 * (c) https://github.com/MontiCore/monticore
 */
export interface ${name?cap_first}Options {
    <#list options as option>
    ${option.getName()}: ${printer.printOptionType(option)};
    </#list>
}

export interface ${name?cap_first}Errors {
    <#list options as option>
    ${option.getName()}: ${printer.printOptionType(option, true)};
    </#list>
}
