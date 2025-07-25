package schematypes._ast;

import schematypes.TypeIdentifier;

public class ASTComponentType extends ASTComponentTypeTOP {

    public ASTComponentType() {
    }

    public ASTComponentType(String type) {
        super(type);
    }

    @Override
    public String toString() {
        return getType();
    }

    @Override
    public Class<?> getNativeType() {
        return Object.class;
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
        return TypeIdentifier.COMPONENT;
    }
}