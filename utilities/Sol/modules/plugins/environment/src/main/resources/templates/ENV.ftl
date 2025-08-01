<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnv>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
ENV <#list partition as variable>
    ${variable.getKey().getValue()}="${variable.getValue().getValue()}"<#if variable_has_next> </#if>
</#list>
</@compress>
