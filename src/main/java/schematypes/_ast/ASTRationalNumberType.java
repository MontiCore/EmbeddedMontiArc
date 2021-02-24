package schematypes._ast;

import schematypes.TypeIdentifier;

import java.util.Optional;

public class ASTRationalNumberType extends ASTRationalNumberTypeTOP {

    public ASTRationalNumberType() {
        super();
    }

    public ASTRationalNumberType(String type, Optional<ASTRange> range, Optional<ASTDomain> values) {
        super(type, range, values);
    }

    @Override
    public String toString() {
        return getType();
    }

    @Override
    public Class<?> getNativeType() {
        return Double.class;
    }

    @Override
    public boolean isEMAType() {
        return true;
    }
    @Override
    public boolean isTypeWithRange() {
        return true;
    }

    @Override
    public boolean isTypeWithDomain() {
        return true;
    }

    @Override
    public TypeIdentifier getSchemaLangType() {
        return TypeIdentifier.RATIONAL_NUMBER;
    }
}