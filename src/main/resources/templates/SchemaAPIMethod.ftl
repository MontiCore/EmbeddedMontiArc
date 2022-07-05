${tc.signature ("schemaMembers","astEnumeratedDeclarationList")}
<#assign typeUtil = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
<#assign enumeratedDeclarations= astEnumeratedDeclarationList>

<#list schemaMembers as schemaMember>
    <#assign methodName = schemaMember.getName()?cap_first><#if typeUtil.isPrimitiveSchemaMember(schemaMember)>
    def get${methodName}(self):
        return self.training_configuration.get${methodName}()
    <#elseif typeUtil.isObjectSchemaMember(schemaMember)>
    <#assign getterMethodName = schemaMember.getType().getGenericType()?cap_first>
    def get${getterMethodName}(self):
        return self.${getterMethodName}(self)
    <#elseif typeUtil.isComplexPropertyDefinition(schemaMember)>
    ${tc.includeArgs("templates.SchemaAPIObjectType",[schemaMember])}
    </#if>
</#list>

<#list enumeratedDeclarations as enumeratedDeclaration>
<#assign methodName = "get"+enumeratedDeclaration.getName()>
    def get${methodName}(self):
        return self.training_configuration.${methodName}()
</#list>