package schematypes._ast;

import conflangliterals._ast.ASTInfinityLiteral;
import de.monticore.mcliterals._ast.ASTSignedNumericLiteral;
import schemalang.validation.TypeCompatibility;

import java.util.Optional;

public class ASTNumberWithInf extends ASTNumberWithInfTOP {

    public ASTNumberWithInf() {
    }

    public ASTNumberWithInf(Optional<ASTSignedNumericLiteral> number, Optional<ASTInfinityLiteral> infinityLiteral, Optional<String> posInf, Optional<String> negInf) {
        super(number, infinityLiteral, posInf, negInf);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("(");
        builder.append(TypeCompatibility.getValueOfLiteral(getNumber()));
        builder.append(")");
        return builder.toString();
    }
}