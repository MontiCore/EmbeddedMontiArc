package schemalang._cocos;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.se_rwth.commons.logging.Log;
import schemalang._ast.ASTTypedDeclaration;
import schemalang._symboltable.ComplexPropertyDefinitionSymbol;
import schemalang._symboltable.SchemaDefinitionScope;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.exception.SchemaLangTechnicalException;
import schematypes.TypeIdentifier;
import schematypes._ast.ASTObjectType;
import schematypes._ast.ASTSchemaType;

import java.util.Optional;

import static schemalang.ErrorCodes.ERROR_CODE_SL_07C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_07C;

public class CustomPropertyDefinitionExists implements SchemaLangASTTypedDeclarationCoCo {

    @Override
    public void check(ASTTypedDeclaration attribute) {
        ASTSchemaType type = attribute.getType();
        if (!TypeIdentifier.OBJECT.equals(type.getSchemaLangType())) {
            return;
        }

        ASTObjectType complexType = (ASTObjectType) attribute.getType();
        String genericType = complexType.getGenericType();

        Optional<? extends Scope> enclosingScopeOpt = attribute.getEnclosingScopeOpt();
        if (!enclosingScopeOpt.isPresent()) {
            // this should not happen
            throw new SchemaLangTechnicalException(String.format("Enclosing scope of property '%s' is not set.", attribute.getName()));
        }

        SchemaDefinitionScope scope = (SchemaDefinitionScope) enclosingScopeOpt.get();
        Optional<? extends ScopeSpanningSymbol> schemaLangDefinitionSymbolOpt = scope.getSpanningSymbol();
        if (!schemaLangDefinitionSymbolOpt.isPresent()) {
            throw new SchemaLangTechnicalException("SchemaLangDefinitionSymbol not available in scope.");
        }

        SchemaDefinitionSymbol schemaDefinitionSymbol = (SchemaDefinitionSymbol) schemaLangDefinitionSymbolOpt.get();
        Optional<ComplexPropertyDefinitionSymbol> attributeDefinitionOpt = schemaDefinitionSymbol.getAttributeEntryOfKindComplex(genericType);

        if (!attributeDefinitionOpt.isPresent()) {
            Log.error(ERROR_CODE_SL_07C.concat(String.format(ERROR_MSG_SL_07C, genericType)),
                    attribute.get_SourcePositionStart());
        }
    }
}