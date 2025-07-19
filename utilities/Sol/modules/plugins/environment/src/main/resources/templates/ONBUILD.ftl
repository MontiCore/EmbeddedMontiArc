<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTOnBuild>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
<#list partition as instruction>
<#assign subInstruction = instruction.getInstruction()>
ONBUILD ${tc.includeArgs("templates/" + subInstruction.getType() + ".ftl", [[subInstruction]])}
</#list>
</@compress>
