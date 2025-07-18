package schemalang._cocos;

import conflangliterals._ast.ASTTypelessLiteral;
import schemalang._ast.ASTSchemaLinkReference;

public class SchemaLinksAreTypeless implements SchemaLangASTSchemaLinkReferenceCoCo {

    @Override
    public void check(ASTSchemaLinkReference node) {
        ASTTypelessLiteral schema = node.getSchema();
    }
}