package schematypes._ast;

import schematypes.TypeIdentifier;

import java.util.Optional;

public class ASTComplexNumberType extends ASTComplexNumberTypeTOP {

    public ASTComplexNumberType() {
    }

    public ASTComplexNumberType(String type, Optional<ASTRange> range, Optional<ASTDomain> values) {
        super(type, range, values);
    }

    @Override
    public Class<?> getNativeType() {
        return null;
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
        return TypeIdentifier.COMPLEX_NUMBER;
    }
}