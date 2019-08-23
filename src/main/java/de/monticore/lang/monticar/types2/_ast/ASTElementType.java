/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.types2._ast;

import de.monticore.lang.monticar.ranges._ast.ASTRange;

import java.util.Optional;

/**
 * @author Christoph Richter
 */
public class ASTElementType extends ASTElementTypeTOP {

    public ASTElementType() {
    }

    public ASTElementType(Optional<String> name, Optional<ASTRange> range) {
        super(name, range);
    }

    public boolean isBoolean() {
        return isPresentName() &&  getName().contentEquals("B") || getName().contentEquals("Boolean");
    }

    public boolean isNaturalNumber() {
        return isPresentName() && getName().contentEquals("N");
    }

    public boolean isWholeNumber() {
        return isPresentName() && getName().contentEquals("Z");
    }

    public boolean isRational() {
        return isPresentName() && getName().contentEquals("Q");
    }

    public boolean isComplex() {
        return isPresentName() && getName().contentEquals("C");
    }

    @Override
    public String getName() {
        if (!super.isPresentName())
            return "Q";
        return super.getName();
    }

}
