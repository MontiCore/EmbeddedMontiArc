<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTRunInstruction>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
RUN <#list partition as instruction><#if instruction.isPresentCommand()>${instruction.getCommand().getValue()}<#elseif instruction.isPresentExecutable()>${instruction.getExecutable().getValue()}<#list instruction.getParameterList() as parameter>${parameter.getValue()}</#list></#if><#if instruction?has_next> && </#if></#list>
</@compress>