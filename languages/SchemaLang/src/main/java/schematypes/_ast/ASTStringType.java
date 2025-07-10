package schematypes._ast;

import schematypes.TypeIdentifier;

import java.util.Optional;

public class ASTStringType extends ASTStringTypeTOP {

    public ASTStringType() {
    }

    public ASTStringType(String type, Optional<ASTDomain> values) {
        super(type, values);
    }

    @Override
    public String toString() {
        return getType();
    }

    @Override
    public Class<?> getNativeType() {
        return String.class;
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
        return true;
    }

    @Override
    public TypeIdentifier getSchemaLangType() {
        return TypeIdentifier.STRING;
    }
}