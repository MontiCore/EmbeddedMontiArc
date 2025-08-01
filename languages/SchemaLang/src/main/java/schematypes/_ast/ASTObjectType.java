package schematypes._ast;

import com.google.common.base.Joiner;
import de.monticore.mcbasictypes1._ast.ASTQualifiedName;
import schematypes.TypeIdentifier;

public class ASTObjectType extends ASTObjectTypeTOP {

    public ASTObjectType() {
    }

    public ASTObjectType(ASTQualifiedName type) {
        super(type);
    }

    public String getGenericType() {
        ASTQualifiedName qualifiedName = getType();
        return Joiner.on('.').join(qualifiedName.getPartList());
    }

    @Override
    public String toString() {
        return getGenericType();
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
        return TypeIdentifier.OBJECT;
    }
}