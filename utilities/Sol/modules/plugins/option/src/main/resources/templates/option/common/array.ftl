<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="options" type="java.util.List<de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol>" -->
<#-- @ftlvariable name="err" type="java.lang.Boolean" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator" -->
<#-- @ftlvariable name="option" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol" -->
<@compress single_line=true>
${tc.signature("option", "err")}
<#assign delegator = glex.getGlobalVar("option.delegator")>
<#assign printer = glex.getGlobalVar("option.printer")>
<#assign options = delegator.getOptionSymbols(option)>
{ <#list options as option>${option.getName()}: ${printer.printOptionType(option, err)}; </#list> }[]
</@compress>