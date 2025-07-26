<#-- (c) https://github.com/MontiCore/monticore -->
<#if (ast.packageName)??>
package ${ast.packageName};
</#if>
package generated;

public class Constants {

<#list ast.basicSchemaProperties as property>
    public static final String ${property.name?upper_case} = "${property.name}";
</#list>

<#list ast.complexPropertyDefinitions as property>
    /*
    * Values for object ${property.name}
    */
    <#list property.allowedValues as value>
    public static final String ${property.name?upper_case}_${value?upper_case} = "${value}";
    </#list>

<#list property.allowedPropertiesForValues?keys as key>
    <#if (property.allowedPropertiesForValues[key]?size != 0)>
    /*
    * Allowed parameters for ${property.name} '${key}'
    */
    <#list property.allowedPropertiesForValues[key] as value>
    public static final String ${property.name?upper_case}_${key?upper_case}_${value?upper_case} = "${value}";
    </#list>

    </#if>
</#list>
</#list>
}