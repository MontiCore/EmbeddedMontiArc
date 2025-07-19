<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTVolume>" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
${tc.signature("partition")}
<@compress single_line=true>
VOLUME [<#list partition as ast><#list ast.getVolumeList() as volume>"${volume.getValue()}"<#if volume_has_next>, </#if></#list><#if ast_has_next>, </#if></#list>]
</@compress>
