package schemalang.validation.cocos;

import com.google.common.base.Joiner;
import conflang._ast.ASTNestedConfigurationEntry;
import conflang._cocos.ConfLangASTNestedConfigurationEntryCoCo;
import conflang._symboltable.NestedConfigurationEntrySymbol;
import schemalang._symboltable.ComplexPropertyDefinitionSymbol;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.TypedDeclarationSymbol;
import schemalang.validation.SchemaViolation;
import schematypes._ast.ASTObjectType;

import java.util.List;
import java.util.Optional;

import static schemalang.ErrorCodes.*;

public class NestedConfigurationEntryIsValid extends AbstractSchemaValidator implements ConfLangASTNestedConfigurationEntryCoCo {

    public NestedConfigurationEntryIsValid(List<SchemaDefinitionSymbol> schemaDefinitionSymbols, List<SchemaViolation> schemaViolations) {
        super(schemaDefinitionSymbols, schemaViolations);
    }

    @Override
    public void check(ASTNestedConfigurationEntry node) {

        Optional<NestedConfigurationEntrySymbol> configurationEntrySymbolOpt = node.getNestedConfigurationEntrySymbolOpt();
        if (!configurationEntrySymbolOpt.isPresent()) {
            throw new RuntimeException("Symbol not set!");
        }
        NestedConfigurationEntrySymbol symbol = configurationEntrySymbolOpt.get();

        // Look for an attribute definition of the form 'loss: loss_type'
        Optional<TypedDeclarationSymbol> attributeDefinitionSymbolOpt = getTypedDeclaration(symbol.getName());

        if (!attributeDefinitionSymbolOpt.isPresent()) {
            if (isNestedEntryDefinedInReferenceModels(symbol)) {
                return;
            }
            String errorMessage = String.format(ERROR_MSG_SL_10C, symbol.getName());
            addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_10C, errorMessage, Joiner.on(", ").join(getAllSchemaNames())));
            logError(node, ERROR_CODE_SL_10C.concat(errorMessage));
            return;
        }

        Optional<ASTObjectType> complexType = Optional.empty();
        TypedDeclarationSymbol simpleAttributeSymbol = attributeDefinitionSymbolOpt.get();
        if (simpleAttributeSymbol.getType() instanceof ASTObjectType) {
            complexType = Optional.of((ASTObjectType) simpleAttributeSymbol.getType());
        }

        // Look if a complex definition with the generic name, e.g. 'loss_type', exists.
        Optional<ComplexPropertyDefinitionSymbol> complexAttributeSymbolOpt = Optional.empty();
        if (complexType.isPresent()) {
            String genericType = complexType.get().getGenericType();
            complexAttributeSymbolOpt = getObjectDefinition(genericType);
        }

        if (!complexAttributeSymbolOpt.isPresent()) {
            ASTObjectType type = complexType.get();
            String errorMessage = String.format(ERROR_MSG_SL_13C, type.getGenericType(), symbol.getName());
            addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_13C, errorMessage, Joiner.on(", ").join(getAllSchemaNames())));
            logError(node, ERROR_CODE_SL_13C.concat(errorMessage));
            return;
        }

        Object value = symbol.getValue();
        ComplexPropertyDefinitionSymbol complexAttributeDefinitionSymbol = complexAttributeSymbolOpt.get();
        if (!complexAttributeDefinitionSymbol.isValidValue((String) value)) {
            String errorMessage = String.format(ERROR_MSG_SL_12C, value, symbol.getName(), Joiner.on(", ").join(complexAttributeDefinitionSymbol.getValidValues()));
            addSchemaViolation(SchemaViolation.create(ERROR_CODE_SL_12C, errorMessage, Joiner.on(", ").join(getAllSchemaNames())));
            logError(node, ERROR_CODE_SL_12C.concat(errorMessage));
        }
    }


}