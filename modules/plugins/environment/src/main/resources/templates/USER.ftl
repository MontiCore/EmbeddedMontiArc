<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTUser>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<#list partition as instruction>
<#assign owner = instruction.getOwner()>
USER ${owner.getUser().getValue()}<#if owner.isPresentGroup()>:${owner.getGroup().getValue()}</#if>
</#list>
