${tc.signature ("ASTTypedDeclaration", "ASTComplexPropertyDefinition")}
<#assign schemaApiUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaApiUtil")>
<#assign objectdeclaration = ASTTypedDeclaration>
<#assign objectdeclarationName = objectdeclaration.getName()>
<#assign typeDefinition = ASTComplexPropertyDefinition>
<#assign declarationTypeName = typeDefinition.getName()>

'''
    possible values:
<#assign allowedValues = typeDefinition.getAllowedValues()>
    <#list allowedValues as value >
        ${value}
    </#list>
'''

def ${schemaApiUtil.createGetterMethodName(objectdeclarationName, "value")}(self):
    return self.training_configuration.${schemaApiUtil.createGetterMethodName(objectdeclarationName,"value")}()

#common hyperparameters
<#list typeDefinition.getSchemaMemberList() as commonParameter>
<#assign methodName = schemaApiUtil.createGetterMethodName(objectdeclarationName, commonParameter.getName())>
def ${methodName}(self):
    return self.training_configuration.${methodName}()
</#list>

#value-specific hyperparameters
<#list typeDefinition.getComplexPropertyValueDefinitionList()  as astComplexPropertyValueDefinition>
    <#list astComplexPropertyValueDefinition.getSchemaMemberList() as astSchemaMember>
        <#assign methodName = schemaApiUtil.createGetterMethodName(objectdeclarationName, astComplexPropertyValueDefinition.getName(), astSchemaMember.getName())>
def ${methodName}(self):
    return self.training_configuration.${schemaApiUtil.createGetterMethodName(objectdeclarationName, astSchemaMember.getName())}()
    </#list>
</#list>
