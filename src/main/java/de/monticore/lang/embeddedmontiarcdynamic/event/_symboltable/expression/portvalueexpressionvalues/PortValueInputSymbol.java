/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantSIUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantValue;
import de.monticore.numberunit.Rationals;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.Optional;

public class PortValueInputSymbol extends PortValueSymbol {

    protected boolean logic = false;
    protected boolean logicValue = false;

    protected boolean number = false;
    protected double numberValue = 0;
    protected boolean numberIsPosInf = false;
    protected boolean numberIsNegInf = false;

    protected boolean variable = false;
    protected String variableValue = null;
    protected boolean variableHasArray = false;
    //protected

    protected Optional<EMAConstantValue> emaValue = Optional.empty();

    public PortValueInputSymbol(){
        super();
    }

    public PortValueInputSymbol(boolean logicValue){
        super();
        this.logic = true;
        this.logicValue = logicValue;
    }

    public PortValueInputSymbol(double numberValue){
        super();
        this.number = true;
        this.numberValue = numberValue;
    }

    public PortValueInputSymbol(double numberValue, boolean posInf, boolean negInf){
        super();
        this.number = true;
        this.numberValue = numberValue;
        this.numberIsPosInf = posInf;
        this.numberIsNegInf = negInf;
    }

    public PortValueInputSymbol(String variableValue){
        this.variable = true;
        this.variableValue = variableValue;
        //TODO handle array
    }

    @Override
    public String getTextualRepresentation() {

        if(this.logic){
            return this.logicValue ? "true" : "false";
        }else if(this.number){
            if(this.numberIsNegInf){
                return "-oo";
            }else if(this.numberIsPosInf){
                return "oo";
            }else{
                return Double.toString(this.numberValue);
            }
        }else if(this.variable){
            return this.variableValue;
            //TODO handle array
        }

        return "";
    }

    //<editor-fold desc="Logic">
    public boolean isLogic() {
        return logic;
    }

    public boolean isLogicValue() {
        return logicValue;
    }
    //</editor-fold>


    //<editor-fold desc="Number">
    public boolean isNumber() {
        return number;
    }

    public double getNumberValue() {
        return numberValue;
    }

    public boolean isNumberIsNegInf() {
        return numberIsNegInf;
    }

    public boolean isNumberIsPosInf() {
        return numberIsPosInf;
    }

    //</editor-fold>

    //<editor-fold desc="Variable">
    public boolean isVariable() {
        return variable;
    }

    public String getVariableValue() {
        return variableValue;
    }

    //</editor-fold>


    public Optional<EMAConstantValue> getEmaValue() {
        return emaValue;
    }

    public void setEmaValue(Optional<EMAConstantValue> emaValue) {
        this.emaValue = emaValue;
    }

    public void setEmaValueWithASTNumberWithUnit(ASTNumberWithUnit node){
        Unit unit = node.getUnit();
        Rational rational = Rationals.doubleToRational((Double)node.getNumber().get());
       this.emaValue = Optional.of(new EMAConstantSIUnit(rational, unit));
    }

    public String getValueStringRepresentation(){
        if(this.isLogic()){
            return this.logicValue ? "true" : "false";
        }
        if(this.variable){
            return this.variableValue;
        }

        if(this.emaValue.isPresent()){
            return this.emaValue.get().getValueAsString();
        }

        return "";
    }

}
