package schematypes._ast;

import schematypes.TypeIdentifier;

public interface ASTSchemaType extends ASTSchemaTypeTOP {

    Class<?> getNativeType();

    boolean isEMAType();

    boolean isTypeWithRange();

    boolean isTypeWithDomain();

    TypeIdentifier getSchemaLangType();
}