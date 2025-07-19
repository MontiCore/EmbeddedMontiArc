package conflangliterals._ast;

import conflangliterals.LiteralHelpers;
import de.monticore.mcliterals._ast.ASTSignedLiteral;

import java.util.List;
import java.util.stream.Collectors;

public class ASTListLiteral extends ASTListLiteralTOP {

    public ASTListLiteral() {
    }

    public ASTListLiteral(List<ASTSignedLiteral> signedLiterals, String value) {
        super(signedLiterals, value);
    }

    @Override
    public String toString() {
        List<ASTSignedLiteral> signedLiterals = getSignedLiteralList();
        String joinedString = signedLiterals.stream().map(signedLiteral ->
                LiteralHelpers.literalToString(signedLiteral)).collect(Collectors.joining(", "));
        return "(".concat(joinedString).concat(")");
    }
}