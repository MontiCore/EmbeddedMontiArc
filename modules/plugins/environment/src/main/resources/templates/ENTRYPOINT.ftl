<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEntryPoint>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
ENTRYPOINT <#list partition as instruction><#assign argument = instruction.getArgument()><#if argument.isPresentCommand()>${argument.getCommand().getValue()}<#elseif argument.isPresentSplit()><#assign split = argument.getSplit()>["${split.getExecutable().getValue()}"<#list split.getParameterList() as parameter>, "${parameter.getValue()}"</#list></#if></#list>]
</@compress>
