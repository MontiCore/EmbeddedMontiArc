package schematypes._ast;

import schematypes.TypeIdentifier;

public class ASTBooleanType extends ASTBooleanTypeTOP {

    public ASTBooleanType() {
        super();
    }

    public ASTBooleanType(String type) {
        super(type);
    }

    @Override
    public String toString() {
        return getType();
    }

    @Override
    public Class<?> getNativeType() {
        return Boolean.class;
    }

    @Override
    public boolean isEMAType() {
        return true;
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
        return TypeIdentifier.BOOLEAN;
    }
}