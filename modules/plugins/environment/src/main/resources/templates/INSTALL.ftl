<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="partition" type="java.util.List<de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstall>" -->
${tc.signature("partition")}
<@compress single_line=true>
RUN apt-get update && apt-get install -y --no-install-recommends <#list partition as ast>
    <#list ast.getPackageList() as package>${package.getValue()} </#list></#list>&& rm -rf /var/lib/apt/lists/*
</@compress>
