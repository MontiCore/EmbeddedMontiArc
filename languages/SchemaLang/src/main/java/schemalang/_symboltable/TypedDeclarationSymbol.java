package schemalang._symboltable;

import schematypes.TypeIdentifier;
import schematypes._ast.ASTSchemaType;

import java.util.Optional;

/**
 *
 */
public class TypedDeclarationSymbol extends TypedDeclarationSymbolTOP {

    private Object defaultValue;
    private TypeIdentifier typeIdentifier;
    private String typeName;
    private ASTSchemaType schemaType;

    public TypedDeclarationSymbol(String name) {
        super(name);
    }

    public Optional<Object> getDefaultValue() {
        return Optional.ofNullable(defaultValue);
    }

    public void setDefaultValue(Object defaultValue) {
        this.defaultValue = defaultValue;
    }

    public TypeIdentifier getSchemaLangType() {
        return typeIdentifier;
    }

    public void setSchemaLangType(TypeIdentifier type) {
        this.typeIdentifier = type;
    }

    public String getTypeName() {
        return typeName;
    }

    public void setTypeName(String typeName) {
        this.typeName = typeName;
    }

    public ASTSchemaType getType() {
        return schemaType;
    }

    public void setType(ASTSchemaType schemaType) {
        this.schemaType = schemaType;
    }
}