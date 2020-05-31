/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolicsolver;

import org.junit.Test;
import org.matheclipse.core.eval.ExprEvaluator;
import org.matheclipse.core.expression.F;
import org.matheclipse.core.interfaces.IAST;
import org.matheclipse.core.interfaces.IExpr;
import org.matheclipse.core.interfaces.ISymbol;
import org.matheclipse.parser.client.SyntaxError;
import org.matheclipse.parser.client.math.MathException;

public class TestSymbolic {

    @Test
    public void test0() {
        ExprEvaluator util = new ExprEvaluator(false, 100);
        String javaForm = util.toJavaForm("a-b");
        System.out.println(javaForm);
        javaForm = util.toJavaForm("a*b^-1");
        System.out.println(javaForm);
        javaForm = util.toJavaForm("a+2");
        System.out.println(javaForm);
        System.out.println(util.eval("0=a+2").toString());
    }

    @Test
    public void test1() {
        try {
            ExprEvaluator util = new ExprEvaluator(false, 100);

            IExpr x1 = util.defineVariable("x1");
            IExpr x2 = util.defineVariable("x2");

            IExpr a1 = F.symbol("a1");
            IExpr a2 = F.symbol("a2");

            IExpr f1 = F.Equal(F.Plus(x1, x2), a1);
            IExpr f2 = F.Equal(x1,F.Times(a2, x2));

            IExpr vars = F.List(x1,x2);
            IExpr functions = F.List(f1,f2);

            IExpr solve = F.Solve(functions, vars);

            IExpr result = util.eval(solve);
            System.out.println(result.toString());
        } catch (SyntaxError e) {
            // catch Symja parser errors here
            System.out.println(e.getMessage());
        } catch (MathException me) {
            // catch Symja math errors here
            System.out.println(me.getMessage());
        } catch (final Exception ex) {
            System.out.println(ex.getMessage());
        } catch (final StackOverflowError soe) {
            System.out.println(soe.getMessage());
        } catch (final OutOfMemoryError oome) {
            System.out.println(oome.getMessage());
        }
    }

    @Test
    public void test2() {
        try {
            ExprEvaluator util = new ExprEvaluator(false, 100);

            ISymbol x = F.symbol("x");
            ISymbol y = util.defineVariable("y");
            ISymbol y2 = util.defineVariable("y2");

            IAST f1 = F.Equal(F.Plus(x,F.Negate(y2)), y);
            IAST f2 = F.Equal(F.Times(y, y), y2);

            IAST vars = F.List(y,y2);
            IAST functions = F.List(f1,f2);

            IAST solve = F.LinearSolve(functions, vars);

            IExpr result = util.eval(solve);
            System.out.println(result.toString());
        } catch (SyntaxError e) {
            // catch Symja parser errors here
            System.out.println(e.getMessage());
        } catch (MathException me) {
            // catch Symja math errors here
            System.out.println(me.getMessage());
        } catch (final Exception ex) {
            System.out.println(ex.getMessage());
        } catch (final StackOverflowError soe) {
            System.out.println(soe.getMessage());
        } catch (final OutOfMemoryError oome) {
            System.out.println(oome.getMessage());
        }
    }
}
