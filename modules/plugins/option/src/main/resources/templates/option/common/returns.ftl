<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="returnType" type="java.lang.String" -->
<#-- @ftlvariable name="err" type="java.lang.Boolean" -->
<@compress single_line=true>
${tc.signature("returnType", "err")}
${returnType}<#if err> | undefined</#if>
</@compress>