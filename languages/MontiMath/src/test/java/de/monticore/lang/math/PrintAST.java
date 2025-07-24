/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.ast.ASTNode;
import de.monticore.lang.math._ast.ASTMathNode;
import de.monticore.lang.math._visitor.MathInheritanceVisitor;
import de.monticore.prettyprint.IndentPrinter;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Optional;

public class PrintAST implements MathInheritanceVisitor {
    IndentPrinter printer = new IndentPrinter();

    protected static String invokeMethod(ASTNode node, String name) {
        Object s = null;
        try {
            Method m = node.getClass().getMethod(name);
            if (m.getDeclaringClass() !=  Object.class) {
                try {
                    s = m.invoke(node);
                } catch (IllegalAccessException e) {
                } catch (InvocationTargetException e) {
                }
            }
        } catch (NoSuchMethodException e) {
        }
        if (s instanceof Optional && ((Optional) s).isPresent())
            s = ((Optional) s).get();
        if (s instanceof String) {
            return (String)s;
        }
        return null;
    }

    @Override
    public void visit(ASTNode node) {
        String s = invokeMethod(node, "getName");
        if (s == null) {
            s = invokeMethod(node, "getSource");
        }
        if (s instanceof String) {
            printer.println(node.getClass().getSimpleName().replace("AST", "") + " " + s);
        }
        else {
            printer.println(node.getClass().getSimpleName().replace("AST", ""));
        }
        printer.indent();
    }

    @Override
    public void endVisit(ASTNode node) {
        printer.unindent();
    }

    public static String printAST(ASTMathNode node) {
        PrintAST printer = new PrintAST();
        node.accept(printer);
        String s = printer.printer.getContent();
        return s;
    }
}
