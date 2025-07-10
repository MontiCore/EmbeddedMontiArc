package conflangliterals._ast;

import java.util.Optional;

public class ASTTypelessLiteral extends ASTTypelessLiteralTOP {

    public ASTTypelessLiteral() {
    }

    public ASTTypelessLiteral(Optional<String> value) {
        super(value);
    }

    @Override
    public String toString() {
        return getValue();
    }
}