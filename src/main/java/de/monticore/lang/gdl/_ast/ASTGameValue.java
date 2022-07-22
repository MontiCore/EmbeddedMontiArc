package de.monticore.lang.gdl._ast;

public class ASTGameValue extends ASTGameValueTOP {
    
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof ASTGameValue) {
            return ((ASTGameValue) obj).getValue().equals(this.getValue());
        }
        return super.equals(obj);
    }

    @Override
    public int hashCode() {
        return getValue().hashCode();
    }

}
