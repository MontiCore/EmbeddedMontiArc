package schematypes._ast;

import de.monticore.mcliterals._ast.ASTIntLiteral;
import de.monticore.mcliterals._ast.ASTLiteral;
import schematypes.TypeIdentifier;

import java.util.List;
import java.util.Optional;

public class ASTMatrixType extends ASTMatrixTypeTOP {

    public ASTMatrixType() {
        super();
    }

    public ASTMatrixType(ASTEMAType type, ASTDimension dimension, Optional<ASTRange> range) {
        super(type, dimension, range);
    }

    @Override
    public String toString() {
        ASTEMAType elementType = getType();
//        Optional<ASTEMAType> emaTypeOpt = elementType.getTypeOpt();
//        if (emaTypeOpt.isPresent()) {
//            ASTEMAType astemaType = emaTypeOpt.get();
//        }

        ASTDimension dimension = getDimension();
        List<ASTLiteral> matrixDimList = dimension.getMatrixDimList();
        StringBuilder builder = new StringBuilder();
        if (matrixDimList != null && !matrixDimList.isEmpty()) {
            builder.append("{");
            for (ASTLiteral astLiteral : matrixDimList) {
                if (astLiteral instanceof ASTIntLiteral) {
                    ASTIntLiteral intLiteral = (ASTIntLiteral) astLiteral;
                    builder.append(intLiteral.getValue());
                }
            }
            builder.append("}");
        }
        return builder.toString();
    }

    @Override
    public Class<?> getNativeType() {
        return Object.class;
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
        return false;
    }

    @Override
    public TypeIdentifier getSchemaLangType() {
        return TypeIdentifier.MATRIX;
    }
}