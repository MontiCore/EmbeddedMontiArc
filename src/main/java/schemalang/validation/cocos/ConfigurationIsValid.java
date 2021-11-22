package schemalang.validation.cocos;

import com.google.common.base.Joiner;
import conflang._ast.ASTConfiguration;
import conflang._cocos.ConfLangASTConfigurationCoCo;
import conflang._symboltable.ConfigurationSymbol;
import schemalang._ast.ASTRequiresRule;
import schemalang._ast.ASTSchemaMember;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.validation.SchemaViolation;

import java.util.List;

import static schemalang.ErrorCodes.*;

public class ConfigurationIsValid extends AbstractSchemaValidator implements ConfLangASTConfigurationCoCo {

    public ConfigurationIsValid(List<SchemaDefinitionSymbol> schemaDefinitionSymbols, List<SchemaViolation> schemaViolations) {
        super(schemaDefinitionSymbols, schemaViolations);
    }

    @Override
    public void check(ASTConfiguration node) {

        checkRequiredParameters(node);
        checkRequiresRules(node);
    }

    private void checkRequiredParameters(ASTConfiguration node) {
        ConfigurationSymbol configurationSymbol = node.getConfigurationSymbol();
        List<ASTSchemaMember> requiredProperties = getAllRequiredProperties();
        for (ASTSchemaMember requiredProperty : requiredProperties) {
            boolean hasRequiredConfigurationEntry = configurationSymbol.containsConfigurationEntry(requiredProperty.getName());
            if (!hasRequiredConfigurationEntry) {
                String schemaNames = Joiner.on(", ").join(getAllSchemaNames());
                String errorMessage = String.format(ERROR_MSG_SL_16C, requiredProperty.getName());
                addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_16C, errorMessage, schemaNames));
                logError(node, ERROR_CODE_SL_16C.concat(errorMessage));
            }
        }
    }

    private void checkRequiresRules(ASTConfiguration node) {
        ConfigurationSymbol configurationSymbol = node.getConfigurationSymbol();
        List<ASTRequiresRule> requiresRules = getAllRequiresRules();
        for (ASTRequiresRule requiresRule : requiresRules) {
            boolean hasConfigurationEntry = configurationSymbol.containsConfigurationEntry(requiresRule.getName());
            if (!hasConfigurationEntry) {
                continue;
            }

            for (String requiredProperty : requiresRule.getDependenciesList()) {
                boolean hasRequiredConfigurationEntry = configurationSymbol.containsConfigurationEntry(requiredProperty);
                if (!hasRequiredConfigurationEntry) {
                    String schemaNames = Joiner.on(", ").join(getAllSchemaNames());
                    String errorMessage = String.format(ERROR_MSG_SL_21C, requiresRule.getName(), requiredProperty);
                    addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_21C, errorMessage, schemaNames));
                    logError(node, ERROR_CODE_SL_21C.concat(errorMessage));
                }
            }
        }
    }
}