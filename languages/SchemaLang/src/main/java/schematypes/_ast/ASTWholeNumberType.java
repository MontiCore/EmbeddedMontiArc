package schematypes._ast;

import schematypes.TypeIdentifier;

import java.util.Optional;

public class ASTWholeNumberType extends ASTWholeNumberTypeTOP {

    public ASTWholeNumberType() {
        super();
    }

    public ASTWholeNumberType(String type, Optional<ASTRange> range, Optional<ASTDomain> values) {
        super(type, range, values);
    }

    @Override
    public String toString() {
        return getType();
    }

    @Override
    public Class<?> getNativeType() {
        return Integer.class;
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
        return TypeIdentifier.WHOLE_NUMBER;
    }
}