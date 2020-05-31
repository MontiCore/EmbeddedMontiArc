/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import java.util.List;

public interface DeterminantSolver {
    public String det(List<? extends List<String>> A);
}
