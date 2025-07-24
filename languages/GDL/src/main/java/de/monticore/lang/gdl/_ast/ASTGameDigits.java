package de.monticore.lang.gdl._ast;

public class ASTGameDigits extends ASTGameDigitsTOP {
    
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof ASTGameDigits) {
            return ((ASTGameDigits) obj).getNumber().getValue() == this.getNumber().getValue();
        }
        return super.equals(obj);
    }

    @Override
    public int hashCode() {
        return Integer.hashCode(getNumber().getValue());
    }

}
