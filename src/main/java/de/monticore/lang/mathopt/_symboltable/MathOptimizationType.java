/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

public enum MathOptimizationType {
    MINIMIZATION,
    MAXIMIZATION;

    /**
     * converts math optimization type to string
     *
     * @return optimization type as string
     */
    @Override
    public String toString() {
        String result = "";
        if (this == MINIMIZATION) {
            result += "minimize";
        } else {
            result += "maximize";
        }
        return result;
    }
}
