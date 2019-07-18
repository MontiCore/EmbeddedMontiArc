<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTShellInstruction>" -->
${tc.signature("partition")}
<@compress single_line=true>
<#list partition as ast>
SHELL ["${ast.getExecutable().getValue()}"<#list ast.getParameterList() as parameter>, "${parameter.getValue()}"</#list>]
</#list>
</@compress>