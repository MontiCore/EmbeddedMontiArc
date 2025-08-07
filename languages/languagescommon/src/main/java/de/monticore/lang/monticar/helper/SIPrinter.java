/* (c) https://github.com/MontiCore/monticore */
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
