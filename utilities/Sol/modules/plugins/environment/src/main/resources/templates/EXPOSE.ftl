<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTExpose>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<#list partition as ast>
EXPOSE ${ast.getPort().getValue()?c}
</#list>
