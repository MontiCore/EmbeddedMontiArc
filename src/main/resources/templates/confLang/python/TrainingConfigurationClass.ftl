${tc.signature ("ASTConfLangCompilationUnit")}

<#assign configurationModel = ASTConfLangCompilationUnit/>
<#assign configurationEntries = configurationModel.getConfiguration().getConfigurationEntryList()/>
class Training_Configuration_${configurationModel.getConfiguration().getName()}:
    def __init__(self):
        pass
<#list configurationEntries as configurationEntry>
    ${tc.includeArgs("templates.confLang.python.Training_Configuration_Method", configurationEntry,[configurationEntry,""])}
</#list>
