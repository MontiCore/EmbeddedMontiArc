/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._ast;

import de.monticore.numberunit._ast.ASTNumberWithUnit;

import java.util.Optional;

/**
 */
public class ASTValueAtTick extends ASTValueAtTickTOP {
    public ASTValueAtTick() {
    }

    public ASTValueAtTick(String name, Optional<ASTNumberWithUnit> value, Optional<ASTNumberWithUnit> lowerBound, Optional<ASTNumberWithUnit> upperBound) {
        super(name, value, lowerBound, upperBound);
    }

    @Override
    public String toString() {
        String result = "";
        result += name + "(";
        if (getValueOpt().isPresent()) {
            result += value.get().getNumber().get().intValue();
        } else {
            if (getLowerBoundOpt().isPresent()) {
                result += lowerBound.get().getNumber().get().intValue();
            }
            if (getUpperBoundOpt().isPresent()) {
                result += ":" + upperBound.get().getNumber().get().intValue();
            }
        }
        result += ")";
        return result;
    }
}
