/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.solutionValidation;

public class PortUpperRangeViewModel {

    private final String name;
    private final String upperLimit;

    public PortUpperRangeViewModel(String name, String upperLimit) {
        this.name = name;
        this.upperLimit = upperLimit;
    }

    public String getName() {
        return name;
    }

    public String getUpperLimit() {
        return upperLimit;
    }
}
