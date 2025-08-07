<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTShell>" -->
${tc.signature("partition")}
<@compress single_line=true>
<#list partition as ast>
<#assign argument = ast.getArgument()>
SHELL ["${argument.getExecutable().getValue()}"<#list argument.getParameterList() as parameter>, "${parameter.getValue()}"</#list>]
</#list>
</@compress>
