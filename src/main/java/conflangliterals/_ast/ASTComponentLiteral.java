package conflangliterals._ast;

import com.google.common.base.Joiner;
import de.monticore.mcbasictypes1._ast.ASTQualifiedName;

public class ASTComponentLiteral extends ASTComponentLiteralTOP {

    public ASTComponentLiteral() {
    }

    public ASTComponentLiteral(ASTQualifiedName value) {
        super(value);
    }

    @Override
    public String toString() {
        return Joiner.on('.').join(getValue().getPartList());
    }
}