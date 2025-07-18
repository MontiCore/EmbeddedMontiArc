/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import java.util.List;
import java.util.Set;

public interface DeterminantSolver {
    public String det(List<? extends List<String>> A);

    void setVariables(Set<String> variables);

    void setConstants(Set<String> constants);
}
