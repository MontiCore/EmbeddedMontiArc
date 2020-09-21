/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ranges._ast;

/**
 */
public class ASTNamedRange extends ASTNamedRangeTOP {

    public ASTNamedRange() {
    }

    public ASTNamedRange(String name) {
        super(name);
    }

    public boolean isNaturalNumbersOneRange() {
        return getName().contentEquals("N1");
    }

    public boolean isNaturalNumbersZeroRange() {
        return getName().contentEquals("N0");
    }

    public boolean isWholeNumbersRange() {
        return getName().contentEquals("Z");
    }
}
