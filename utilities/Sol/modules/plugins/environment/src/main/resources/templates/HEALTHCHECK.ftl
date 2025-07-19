<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTHealthCheck>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<#list partition as instruction>
HEALTHCHECK <#if instruction.isPresentCommand()><#list instruction.getOptionList() as option>${option.getOption()}=${option.getValue().getValue()} </#list>CMD ${instruction.getCommand().getValue()}<#else>NONE</#if>
</#list>
