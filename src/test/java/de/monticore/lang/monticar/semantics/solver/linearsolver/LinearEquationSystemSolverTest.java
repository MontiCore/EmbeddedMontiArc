package de.monticore.lang.monticar.semantics.solver.linearsolver;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;

import java.io.IOException;
import java.io.StringReader;
import java.util.Optional;

import static org.junit.Assert.fail;

public class LinearEquationSystemSolverTest {

//    @Test
//    public void solveLinearEquationSystem1() {
//        ASTExpression expression1 = parseExpression("x2=a1-x1");
//        ASTExpression expression2 = parseExpression("x1=a2*x2");
//
//        Set<ASTExpression> system = new HashSet<>();
//        Set<String> variables = new HashSet<>();
//        system.add(expression1);
//        system.add(expression2);
//        variables.add("x1");
//        variables.add("x2");
//
//        MathEclipseSolver detSimplsolver = new MathEclipseSolver();
//        LinearEquationSystemSolver solver = new LinearEquationSystemSolver(detSimplsolver, detSimplsolver);
//
////        Map<String, String> res = solver.solve(system, variables);
////
////        assertEquals(res.get("x1"), "(a1*a2)/(1+a2)");
////        assertEquals(res.get("x2"), "a1/(1+a2)");
//    }
//
//    @Test
//    public void solveLinearEquationSystem2() {
//        ASTExpression expression1 = parseExpression("x2=a1-a2*x2");
//
//        Set<ASTExpression> system = new HashSet<>();
//        Set<String> variables = new HashSet<>();
//        system.add(expression1);
//        variables.add("x2");
//
//        MathEclipseSolver detSimplsolver = new MathEclipseSolver();
//        LinearEquationSystemSolver solver = new LinearEquationSystemSolver(detSimplsolver, detSimplsolver);
//
////        Map<String, String> res = solver.solve(system, variables);
////
////        assertEquals(res.get("x2"), "a1/(1+a2)");
//    }

    EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();

    private ASTExpression parseExpression(String s) {
        Optional<ASTExpression> expression = null;
        try {
            expression = parser.parseExpression(new StringReader(s));
        } catch (IOException e) {
            fail(e.getMessage());
        }

        if (!expression.isPresent())
            fail();

        return expression.get();
    }
}