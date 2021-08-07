package de.monticore.lang.gdl._ast;

public class ASTGameExpression extends ASTGameExpressionTOP {

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("(");
        sb.append(this.getType().toString());
        for (ASTGameRelation arg : this.getArgumentsList()) {
            sb.append(" " + arg.toString());
        }
        sb.append(")");
        return sb.toString();
    }

}