<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTWorkDir>" -->
${tc.signature("partition")}
<@compress single_line=true>
<#list partition as ast>WORKDIR "${ast.getDirectory().getValue()}"</#list>
</@compress>
