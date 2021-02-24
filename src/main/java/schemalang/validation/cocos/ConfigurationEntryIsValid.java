package schemalang.validation.cocos;

import com.google.common.base.Joiner;
import conflang._ast.ASTConfigurationEntry;
import conflang._cocos.ConfLangASTConfigurationEntryCoCo;
import conflang._symboltable.ConfigurationEntrySymbol;
import schemalang._symboltable.*;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.SchemaViolation;
import schematypes.TypeIdentifier;

import java.util.List;
import java.util.Optional;

import static schemalang.ErrorCodes.*;

public class ConfigurationEntryIsValid extends AbstractSchemaValidator implements ConfLangASTConfigurationEntryCoCo {

    public ConfigurationEntryIsValid(List<SchemaDefinitionSymbol> schemaDefinitionSymbols, List<SchemaViolation> schemaViolations) {
        super(schemaDefinitionSymbols, schemaViolations);
    }

    @Override
    public void check(ASTConfigurationEntry node) {

        if (node.isNestedConfiguration()) {
            return;
        }

        Optional<ConfigurationEntrySymbol> simpleConfigurationEntrySymbolOpt = node.getConfigurationEntrySymbolOpt();
        if (!simpleConfigurationEntrySymbolOpt.isPresent()) {
            throw new SchemaLangTechnicalException("Symbol not set!");
        }
        ConfigurationEntrySymbol symbol = simpleConfigurationEntrySymbolOpt.get();

        Optional<TypedDeclarationSymbol> simpleAttributeSymbolOpt = getTypedDeclaration(symbol.getName());
        Optional<EnumeratedDeclarationSymbol> enumAttributeSymbolOpt = Optional.empty();
        if (!simpleAttributeSymbolOpt.isPresent()) {
            // Check for enum definition
            enumAttributeSymbolOpt = getAttributeEntryOfKindEnum(symbol.getName());
        }

        Optional<NestedSchemaEnumLinkDefinitionSymbol> schemaEnumLinkDefinitionSymbol = Optional.empty();
        if (!simpleAttributeSymbolOpt.isPresent() && !enumAttributeSymbolOpt.isPresent()) {
            // Check for enum definition
            schemaEnumLinkDefinitionSymbol = getAttributeEntryOfKindEnumLinkDefinition(symbol.getName());
        }

        if (!simpleAttributeSymbolOpt.isPresent() && !enumAttributeSymbolOpt.isPresent() && !schemaEnumLinkDefinitionSymbol.isPresent()) {
            // Last chance: check complex attribute definition
            simpleAttributeSymbolOpt = findBasicPropertyInObjectTypeDefinitions(node.getEnclosingScope(), symbol);
            enumAttributeSymbolOpt = findEnumPropertyInObjectTypeDefinitions(node.getEnclosingScope(), symbol);
        }

        if (!simpleAttributeSymbolOpt.isPresent() && !enumAttributeSymbolOpt.isPresent() && !schemaEnumLinkDefinitionSymbol.isPresent()) {
            if (!isDefinedInReferenceModel(symbol.getName())) {
                String schemaNames = Joiner.on(", ").join(getAllSchemaNames());
                String errorMessage = String.format(ERROR_MSG_SL_10C, symbol.getName());
                addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_10C, errorMessage, schemaNames));
                logError(node, ERROR_CODE_SL_10C.concat(errorMessage));
                return;
            }
        }

        if (simpleAttributeSymbolOpt.isPresent()) {
            TypedDeclarationSymbol simpleAttributeSymbol = simpleAttributeSymbolOpt.get();
            if (TypeIdentifier.OBJECT.equals(simpleAttributeSymbol.getSchemaLangType())) {
                Optional<ComplexPropertyDefinitionSymbol> complexAttributeSymbolOpt = getObjectDefinition(simpleAttributeSymbol.getTypeName());
                if (!complexAttributeSymbolOpt.isPresent()) {
                    String schemaNames = Joiner.on(", ").join(getAllSchemaNames());
                    String errorMessage = String.format(ERROR_MSG_SL_13C, simpleAttributeSymbol.getTypeName(), symbol.getName());
                    addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_13C, errorMessage, schemaNames));
                    logError(node, ERROR_CODE_SL_13C.concat(errorMessage));
                }
            }
            checkTypedDeclaration(node, simpleAttributeSymbol);

        } else if (enumAttributeSymbolOpt.isPresent()) {
            EnumeratedDeclarationSymbol enumAttributeSymbol = enumAttributeSymbolOpt.get();
            checkEnumAttributeDefinition(node, enumAttributeSymbol);
        }
    }
}