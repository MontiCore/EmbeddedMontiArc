${tc.signature ("ASTSchemaDefinition","astEnumeratedDeclarationList")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign schemaUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaUtil")>
<#assign schemaApiUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaApiUtil")>
<#assign enumeratedDeclarations= astEnumeratedDeclarationList>
<#assign schemaDefinition = ASTSchemaDefinition>
<#assign schemaMembers = schemaDefinition.getSchemaMemberList()>

<#list schemaMembers as schemaMember>
    <#assign methodName = schemaApiUtil.createGetterMethodName(schemaMember.getName())>
<#if typeUtil.isPrimitiveSchemaMember(schemaMember)>
    def ${methodName}(self):
        return self.training_configuration.${methodName}()
    <#elseif typeUtil.isObjectSchemaMember(schemaMember)>
    <#assign complexPropertyDefinition = schemaUtil.getPropertyDefinitionForDeclaration(schemaMember,schemaDefinition)>
    ${tc.includeArgs("templates.SchemaAPI_ObjectType",[schemaMember, complexPropertyDefinition])}
    </#if>
</#list>

<#list enumeratedDeclarations as enumeratedDeclaration>
<#assign methodName = schemaApiUtil.createGetterMethodName(enumeratedDeclaration.getName())>
    def ${methodName}(self):
        return self.training_configuration.${methodName}()
</#list>