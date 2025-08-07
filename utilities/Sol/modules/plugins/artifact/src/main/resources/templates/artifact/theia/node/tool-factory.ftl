<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tool" type="de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "tool")}//tslint:disable
<#assign isVirtual = tool.isVirtual()>
<#assign name = tool.getName()>
export const ${name}<#if isVirtual>Virtual</#if>ToolFactory = Symbol("${name}<#if isVirtual>Virtual</#if>ToolFactory");
