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
//package de.monticore.lang.montiarc.helper;
//
//import java.util.Stack;
//import java.util.stream.Collectors;
//
//import de.monticore.lang.monticar.si._ast.ASTEMADenominator;
//import de.monticore.lang.monticar.si._ast.ASTEMANumerator;
//import de.monticore.lang.monticar.si._ast.ASTEMAUnit;
//import de.monticore.lang.monticar.si._visitor.SIVisitor;
//import de.monticore.literals.literals._ast.ASTNumericLiteral;
//import de.monticore.literals.literals._ast.ASTSignedIntLiteral;
//import de.monticore.literals.prettyprint.LiteralsPrettyPrinterConcreteVisitor;
//import de.monticore.prettyprint.IndentPrinter;
//
///**
// * Created by MichaelvonWenckstern on 27.01.2017.
// */
//public class SIPrinter implements SIVisitor {
//  private Stack<String> s = new Stack<>();
//
//  // this is needed due to a bug in MontiCore
//  public static class LiteralsPrinter extends LiteralsPrettyPrinterConcreteVisitor {
//    public LiteralsPrinter() {
//      super(new IndentPrinter());
//    }
//
//    @Override
//    public void visit(ASTSignedIntLiteral l) {
//      if (l.isNegative()) {
//        this.printer.print("-");
//      }
//      super.visit(l);
//    }
//  }
//
//  private static String printLiteral(ASTNumericLiteral astNumericLiteral) {
//    return new LiteralsPrinter().prettyprint(astNumericLiteral);
//  }
//
//  public static String printUnitAST(ASTEMAUnit unit) {
//    SIPrinter p = new SIPrinter();
//    p.handle(unit);
//    return p.s.stream().map(Object::toString).collect(Collectors.joining());
//  }
//
//  @Override
//  public void visit(ASTEMANumerator numerator) {
//    if (numerator.isSTAR()) {
//      s.push("*");
//    }
//
//    s.push(numerator.getNumerator());
//
//    if (numerator.getExponentOpt().isPresent()) {
//      s.push("^");
//      s.push(printLiteral(numerator.getExponentOpt().get()));
//    }
//  }
//
//  @Override
//  public void visit(ASTEMADenominator denominator) {
//    if (denominator.isSLASH()) {
//      s.push("/");
//    }
//    s.push(denominator.getDenominator());
//
//    if (denominator.getDenominatorExponentOpt().isPresent()) {
//      s.push("^");
//      s.push(printLiteral(denominator.getDenominatorExponentOpt().get()));
//    }
//  }
//}
