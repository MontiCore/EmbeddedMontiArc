<#-- (c) https://github.com/MontiCore/monticore -->
<#assign helper = glex.getGlobalVar("helper")>
package ${helper.getPackage()};

script ${helper.getName()}
${helper.getBody()}
end
