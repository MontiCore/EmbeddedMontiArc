package schematypes._ast;

import schematypes.TypeIdentifier;

import java.util.List;

public class ASTListType extends ASTListTypeTOP {

    public ASTListType() {
        super();
    }

    public ASTListType(ASTSchemaType schemaType, String type) {
        super(schemaType, type);
    }

    @Override
    public String toString() {
        return getSchemaType().toString().concat(getType());
    }

    @Override
    public Class<?> getNativeType() {
        return List.class;
    }

    @Override
    public boolean isEMAType() {
        return false;
    }

    @Override
    public boolean isTypeWithRange() {
        return false;
    }

    @Override
    public boolean isTypeWithDomain() {
        return false;
    }

    @Override
    public TypeIdentifier getSchemaLangType() {
        return TypeIdentifier.LIST;
    }
}