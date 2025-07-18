<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTRun>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
RUN <#list partition as instruction><#assign argument = instruction.getArgument()><#if argument.isPresentCommand()>${argument.getCommand().getValue()}<#elseif argument.isPresentSplit()><#assign split = argument.getSplit()>${split.getExecutable().getValue()}<#list split.getParameterList() as parameter>${parameter.getValue()}</#list></#if><#if instruction?has_next> && </#if></#list>
</@compress>
