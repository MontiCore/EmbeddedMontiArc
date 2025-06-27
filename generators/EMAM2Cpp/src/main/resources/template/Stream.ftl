<#-- (c) https://github.com/MontiCore/monticore -->
package ${streamTest.packageName};

stream ${streamTest.testName} for ${streamTest.componentName}{
<#list streamTest.ports as p>
${p.name}:<#list p.values as v> ${v.getStringRepresentation()} <#if v_has_next> tick </#if>></#list>;
</#list>
};

#endif
