/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

public class PortRangeValidationViewModel {

    private final String name;
    private final String upperLimitValidation;
    private final String lowerLimitValidation;

    public PortRangeValidationViewModel(String name, String upperLimitValidation, String lowerLimitValidation) {
        this.name = name;
        this.upperLimitValidation = upperLimitValidation;
        this.lowerLimitValidation = lowerLimitValidation;
    }

    public String getName() {
        return name;
    }

    public String getUpperLimitValidation() {
        return upperLimitValidation;
    }

    public String getLowerLimitValidation() {
        return lowerLimitValidation;
    }
}
