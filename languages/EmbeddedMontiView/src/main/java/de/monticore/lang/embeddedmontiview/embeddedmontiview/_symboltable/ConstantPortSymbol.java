/* (c) https://github.com/MontiCore/monticore */
//
//package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;
//
//
//import de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant.*;
//
////import de.monticore.literals.literals._ast.*;
//import de.monticore.lang.monticar.literals2._ast.*;
//import de.se_rwth.commons.logging.Log;
//import org.jscience.mathematics.number.Rational;
//
//import javax.measure.unit.Unit;
//
//import siunit.monticoresiunit.si._ast.ASTUnitNumber;
//
///**
// * The ConstantPortSymbol is a port which has a constant value assigned and is used
// * by a ConstantConnector to connect this value to other ports.
// *
// */
//public class ConstantPortSymbol extends ViewPortSymbol {
//    EMAConstantValue constantValue;
//
//    /**
//     * use this constructor for automatic naming of constant ports
//     */
//    public ConstantPortSymbol() {
//        super(ConstantPortSymbol.getNextConstantPortName());
//    }
//
//    public ConstantPortSymbol(String name) {
//        super(name);
//    }
//
//
//    public EMAConstantValue getConstantValue() {
//        return constantValue;
//    }
//
//
//    public void setConstantValue(EMAConstantValue value) {
//        Log.debug("" + value.getValue().toString(), "value setting");
//        this.constantValue = value;
//    }
//
//
//    /**
//     * initializes ConstantPort from a UnitNumberLiteral
//     */
//    public void initConstantPortSymbol(ASTUnitNumber si_unit) {
//        Unit unit = si_unit.getUnit().get();
//        Rational rational = si_unit.getNumber().get();
//
//        setConstantValue(new EMAConstantSIUnit(rational, unit));
//    }
//
//    /**
//     * initializes ConstantPort from a BooleanLiteral
//     */
//    public void initConstantPortSymbol(ASTBooleanLiteral astBooleanLiteral) {
//        setConstantValue(new EMAConstantBoolean(astBooleanLiteral.getSource() == 1));
//    }
//
//    private static int lastID = 1;
//
//    public static String getNextConstantPortName() {
//        return "CONSTANTPORT" + lastID++;
//    }
//
//    @Override
//    public boolean isConstant() {
//        return true;
//    }
//
//    public static ConstantPortSymbol createConstantPortSymbol(de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTConnector node) {
//        ConstantPortSymbol constantPortSymbol = new ConstantPortSymbol();
//
//        if (node.getUnitNumberResolution().isPresent()) {
//            constantPortSymbol.initConstantPortSymbol(node.getUnitNumberResolution().get().getUnitNumber().get());
//        } else if (node.getBoolLiteral().isPresent()) {
//            constantPortSymbol.initConstantPortSymbol(node.getBoolLiteral().get());
//        }
//
//        return constantPortSymbol;
//    }
//}
