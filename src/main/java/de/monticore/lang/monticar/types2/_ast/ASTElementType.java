/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.types2._ast;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRangeStepResolution;
import de.monticore.lang.monticar.ranges._ast.ASTRangeTOP;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;

import java.util.Optional;

/**
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
        return isPresentName() && getName().contentEquals("Z") || !isPresentName() && hasIntegerStep();
    }

    private boolean hasIntegerStep() {
        if(range.isPresent() && !range.get().isPresentStep()){
            return true;
        }

        Optional<Double> stepValueOpt = range
                .flatMap(ASTRangeTOP::getStepOpt)
                .flatMap(ASTRangeStepResolution::getUnitNumberResolutionOpt)
                .flatMap(ASTUnitNumberResolution::getNumber);

        boolean hasIntegerStep = false;
        if(stepValueOpt.isPresent()){
            double stepValue = stepValueOpt.get();
            if((stepValue == Math.floor(stepValue)) && !Double.isInfinite(stepValue)){
                hasIntegerStep = true;
            }
        }
        return hasIntegerStep;
    }

    public boolean isRational() {
        return isPresentName() && getName().contentEquals("Q") || !isPresentName() && !hasIntegerStep();
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
