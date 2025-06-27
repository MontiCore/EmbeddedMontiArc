/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import java.util.Set;

public interface Simplifier {
    public String simplify(String expression);

    void setVariables(Set<String> variables);

    void setConstants(Set<String> constants);
}
