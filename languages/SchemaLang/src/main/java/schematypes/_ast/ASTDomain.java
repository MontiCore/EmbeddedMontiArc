package schematypes._ast;

import de.monticore.mcliterals._ast.ASTSignedLiteral;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import static schemalang.validation.TypeCompatibility.getValueOfLiteral;

public class ASTDomain extends ASTDomainTOP {

    public ASTDomain() {
    }

    public ASTDomain(List<ASTSignedLiteral> valuess) {
        super(valuess);
    }

    @Override
    public String toString() {
        List<ASTSignedLiteral> values = getValuesList();
        return "<" +
                values.stream().map(v -> Objects.requireNonNull(getValueOfLiteral(v)).toString())
                        .collect(Collectors.joining(", ")) +
                ">";
    }
}