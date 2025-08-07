/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import de.monticore.lang.monticar.semantics.util.execution.ExecuteZ3;
import de.monticore.lang.monticar.semantics.underspecification.SolveResult;
import de.monticore.lang.monticar.semantics.util.parse.Z3Parser;
import org.junit.Test;

public class Z3Test {
    @Test
    public void test1() {
        String testz3 = ExecuteZ3.executeZ3("src.test.resources.testing.test.z3");
        System.out.println(testz3);
        SolveResult parse = Z3Parser.parseSolve(testz3);
        parse.toString();
    }
}
