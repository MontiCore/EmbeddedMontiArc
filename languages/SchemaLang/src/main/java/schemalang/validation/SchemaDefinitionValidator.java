package schemalang.validation;

import com.google.common.collect.Lists;
import conflang._ast.ASTConfiguration;
import conflang._cocos.ConfLangCoCoChecker;
import conflang._symboltable.ConfigurationSymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.cocos.ConfigurationEntryIsValid;
import schemalang.validation.cocos.ConfigurationIsValid;
import schemalang.validation.cocos.NestedConfigurationEntryIsValid;

import java.util.List;
import java.util.Optional;

public class SchemaDefinitionValidator {

    public static List<SchemaViolation> validateConfiguration(List<SchemaDefinitionSymbol> schemaDefinitionSymbols,
                                             ConfigurationSymbol configurationSymbol) {

        Optional<ASTConfiguration> confLangConfigurationNode = configurationSymbol.getConfigurationNode();
        if (!confLangConfigurationNode.isPresent()) {
            throw new SchemaLangTechnicalException(String.format("AST node for configuration '%s' not set.", configurationSymbol.getName()));
        }

        ConfLangCoCoChecker checker = new ConfLangCoCoChecker();
        List<SchemaViolation> schemaFindingsConfiguration = Lists.newArrayList();
        checker.addCoCo(new ConfigurationIsValid(schemaDefinitionSymbols, schemaFindingsConfiguration));

        List<SchemaViolation> schemaFindingsSimpleConfigurationEntries = Lists.newArrayList();
        checker.addCoCo(new ConfigurationEntryIsValid(schemaDefinitionSymbols, schemaFindingsSimpleConfigurationEntries));

        List<SchemaViolation> schemaFindingsNestedConfigurationEntries = Lists.newArrayList();
        checker.addCoCo(new NestedConfigurationEntryIsValid(schemaDefinitionSymbols, schemaFindingsNestedConfigurationEntries));
        checker.checkAll(confLangConfigurationNode.get());

        List<SchemaViolation> schemaViolations = Lists.newArrayList();
        schemaViolations.addAll(schemaFindingsConfiguration);
        schemaViolations.addAll(schemaFindingsSimpleConfigurationEntries);
        schemaViolations.addAll(schemaFindingsNestedConfigurationEntries);
        return schemaViolations;
    }
}