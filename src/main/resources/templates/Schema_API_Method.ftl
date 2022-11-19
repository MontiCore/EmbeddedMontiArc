${tc.signature ("ASTSchemaDefinition","astEnumeratedDeclarationList")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign schemaUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil")>
<#assign enumeratedDeclarations= astEnumeratedDeclarationList>
<#assign schemaDefinition = ASTSchemaDefinition>
<#assign schemaMembers = schemaDefinition.getSchemaMemberList()>

<#list schemaMembers as schemaMember>
    <#assign methodName = schemaMember.getName()><#if typeUtil.isPrimitiveSchemaMember(schemaMember)>
    def get_${methodName}(self):
        return self.training_configuration.get_${methodName}()
    <#elseif typeUtil.isObjectSchemaMember(schemaMember)>
    <#assign complexPropertyDefinition = schemaUtil.getPropertyDefinitionForDeclaration(schemaMember,schemaDefinition)>
    ${tc.includeArgs("templates.SchemaAPI_ObjectType",[schemaMember, complexPropertyDefinition])}
    </#if>
</#list>

<#list enumeratedDeclarations as enumeratedDeclaration>
<#assign methodName = "get_"+enumeratedDeclaration.getName()>
    def ${methodName}(self):
        return self.training_configuration.${methodName}()
</#list>