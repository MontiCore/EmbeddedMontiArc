<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTAdd>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<#list partition as instruction>
ADD <#assign operation = instruction.getOperation()><#if operation.isPresentOwner()><#assign owner = operation.getOwner()>--chown=${owner.getUser().getValue()}<#if owner.isPresentGroup()>:${operation.getOwner().getGroup().getValue()}</#if></#if><#list operation.getSourceList() as source>"${source.getValue()}" </#list>"${operation.getDestination().getValue()}"
</#list>
