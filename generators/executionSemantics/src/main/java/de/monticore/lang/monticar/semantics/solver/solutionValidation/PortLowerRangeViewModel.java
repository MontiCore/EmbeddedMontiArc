/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

public class PortLowerRangeViewModel {

    private final String name;
    private final String lowerLimit;

    public PortLowerRangeViewModel(String name, String lowerLimit) {
        this.name = name;
        this.lowerLimit = lowerLimit;
    }

    public String getName() {
        return name;
    }

    public String getLowerLimit() {
        return lowerLimit;
    }
}
