package de.monticore.lang.gdl._ast;

public class ASTGameTuple extends ASTGameTupleTOP {
    
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof ASTGameTuple) {
            return ((ASTGameTuple) obj).getElementList().equals(this.getElementList());
        }
        return super.equals(obj);
    }

    @Override
    public int hashCode() {
        return getElementList().hashCode();
    }

}
