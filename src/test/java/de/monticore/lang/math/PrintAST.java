/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
