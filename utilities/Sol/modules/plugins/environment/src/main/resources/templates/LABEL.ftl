<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTLabel>" -->
${tc.signature("partition")}
LABEL <#list partition as ast>"${ast.getKey().getValue()}" = "${ast.getValue().getValue()}"<#if ast_has_next> </#if></#list>
