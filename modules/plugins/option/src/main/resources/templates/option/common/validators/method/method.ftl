<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="symbol" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions" -->
<#-- @ftlvariable name="option" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="suffix" type="java.lang.String" -->
${tc.signature("symbol", "option", "suffix")}
<#assign delegator = glex.getGlobalVar("option.delegator")>
<#assign printer = glex.getGlobalVar("option.printer")>
<#assign name = symbol.getName()>
<#assign optionName = option.getName()>
public validate${suffix}(${optionName}: ${printer.printOptionType(option)}, options: ${name?cap_first}Options): Promise<${printer.printOptionType(option, true)}> {
    ${printer.printBody(option)}
}