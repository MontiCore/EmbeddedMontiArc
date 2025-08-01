/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import de.monticore.lang.monticar.semantics.util.execution.ExecuteZ3;
import org.junit.Test;

public class Z3Test {
    @Test
    public void test1() {
        String testz3 = ExecuteZ3.executeZ3("test.z3");
        System.out.println(testz3);
    }
}
