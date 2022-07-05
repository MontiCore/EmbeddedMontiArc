${tc.signature ("schemaDefinition","superSchema","schemaMembers")}
<#assign typeHelper = tc.instantiate("de.monticore.lang.monticar.cnnarch.generator.util.SchemaTypeUtil")>
class ${schemaDefinition.getName()}<#if superSchema??>(${superSchema.getName()}) </#if>:

    def __init__(self):
        self.training_configuration = None
    def set_training_configuration(self,training_configuration):
        self.training_configuration =  training_configuration
    ${tc.includeArgs("templates.SchemaAPIMethod",[schemaMembers, schemaDefinition.getEnumPropertyDefinitions()])}
