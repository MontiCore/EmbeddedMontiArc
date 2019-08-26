/* (c) https://github.com/MontiCore/monticore */
package de.monticore.numberunit._ast;

import de.monticore.literals.literals._ast.ASTNumericLiteral;
import de.monticore.numberunit._parser.NumberUnitParser;
import de.se_rwth.commons.logging.Log;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;
import javax.measure.unit.Unit;
import java.io.IOException;
import java.util.Optional;

import static de.monticore.numberunit.PrintHelper.print;

/**
 * Created by MichaelvonWenckstern on 08.01.2018.
 */
public class ASTNumberWithUnit extends ASTNumberWithUnitTOP {

    public ASTNumberWithUnit(
            Optional<ASTComplexNumber> complexNumber,
            Optional<ASTNumberWithInf> number,
            Optional<ASTUnit> unit) {
        super(complexNumber, number, unit);
    }

    public ASTNumberWithUnit() {
        super();
    }

    public boolean isPlusInfinite() {
        return this.isPresentNum() && this.getNum().isPresentInf();
    }

    public boolean isMinusInfinite() {
        return this.isPresentNum() && this.getNum().isPresentNegInf();
    }

    public boolean isComplexNumber() {
        return this.isPresentCn();
    }

    /**
     * returns Optional.empty() if the number is:
     * a) a complex number
     * b) plus or minus infinity
     *
     * @return
     */
    public Optional<Double> getNumber() {
        if (this.isPresentNum() && this.getNum().isPresentNumber()) {
            ASTNumericLiteral number = this.getNum().getNumber();
            double d;
            if (this.getNum().isPresentNegNumber()) {
                d = -1 * Double.parseDouble(print(number));
            } else {
                d = Double.parseDouble(print(number));
            }

            if (this.isPresentUn() && this.getUn().isPresentImperialUnit() &&
                    this.getUn().getImperialUnit().getName().equals("th")) {
                d *= 0.0254;
            }
            return Optional.of(d);
        }
        return Optional.empty();
    }

    public void setNumber(Double doubleValue) {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            this.setNumOpt(parser.parse_StringNumberWithInf(Double.toString(doubleValue)));
        } catch (IOException e) {
            Log.error("Can not read double value", e);
        }
    }

    public Optional<ASTComplexNumber> getComplexNumber() {
        return this.getCnOpt();
    }

    public Unit getUnit() {
        if (this.isPresentUn()) {
            if (this.getUn().isPresentDegCelsius()) {
                return SI.CELSIUS;
            }
            if (this.getUn().isPresentDegFahrenheit()) {
                return NonSI.FAHRENHEIT;
            }
            if (this.getUn().isPresentImperialUnit()) {
                if (this.getUn().getImperialUnit().getName().equals("th")) {
                    return Unit.valueOf("mm");
                }
                return Unit.valueOf(this.getUn().getImperialUnit().getName());
            }
            if (this.getUn().isPresentSIUnit()) {
                return siUnit(this.getUn().getSIUnit());
            }
        }
        return Unit.ONE;
    }

    public void setUnit(Unit unit) {
        NumberUnitParser parser = new NumberUnitParser();
        try {
            if (!unit.toString().isEmpty())
                setUnOpt(parser.parse_StringUnit(unit.toString()));
            else
                setUnOpt(Optional.empty());
        } catch (IOException e) {
            Log.error(String.format("Can not read unit %s", unit.toString()), e);
        }
    }

    protected Unit siUnit(ASTSIUnit siUnit) {
        if (!siUnit.isEmptyTimeDivs() && siUnit.isPresentSiUnitDimensionless()) {
            return Unit.valueOf(siUnit.getSiUnitDimensionless().getName().replace("deg", "°"));
        }
        String s = toString(siUnit.getSIUnitBasicList().get(0));
        for (int i = 1; i < siUnit.getSIUnitBasicList().size(); i++) {
            s += toString(siUnit.getTimeDiv(i - 1)) + toString(siUnit.getSIUnitBasicList().get(i));
        }
        return Unit.valueOf(s);
    }

    protected String toString(ASTSIUnitBasic sib) {
        String unit = "";
        if (sib.isPresentUnitBaseDimWithPrefix()) {
            unit = sib.getUnitBaseDimWithPrefix().getName();
        } else if (sib.isPresentOfficallyAcceptedUnit()) {
            unit = sib.getOfficallyAcceptedUnit().getName();
        } else if (sib.isPresentDeg()) {
            unit = "°";
        }
        if (sib.isPresentSignedIntLiteral()) {
            unit = unit + "^" + print(sib.getSignedIntLiteral());
        }
        return unit;
    }

    protected String toString(ASTTimeDiv timeDivs) {
        return timeDivs.isPresentIsDiv() ? "/" : "*";
    }
}
