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
package de.monticore.lang.numberunit._ast;

import de.monticore.lang.numberunit.Rationals;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import org.jscience.physics.amount.Amount;

import javax.measure.unit.Unit;
import java.util.Optional;

/**
 * Created by michaelvonwenckstern on 05.02.17.
 *
 * should be the same as org.jscience.physics.amount.Amount only with Rational instead of double
 * the methods plus, ... must be implemented later on
 *
 * TODO implement missing methods
 */
public class ASTUnitNumber extends ASTUnitNumberTOP {

    protected ASTUnitNumber() {
        super();
    }

    protected ASTUnitNumber(String tUnitNumber, String tUnitInf) {
        super(tUnitNumber, tUnitInf);
    }

    protected Rational number = Rational.ZERO;
    protected Unit unit = Unit.ONE;
    protected Boolean infSign = true;


    public ASTUnitNumber(Rational number, Unit unit) {
        this.number = number;
        this.unit = unit;

        // do not call setter to avoid infinity loop
        this.tUnitNumber = Optional.of(String.format("%s %s", number, unit));
    }

    public ASTUnitNumber(Boolean infSign, Unit unit) {
        this.infSign = infSign;
        this.unit = unit;

        // do not call setter to avoid infinity loop
        String signString;
        if(infSign) {
            signString = "+oo ";
        } else {
            signString = "-oo ";
        }
        this.tUnitInf = Optional.of(String.format("%s %s", signString, unit));
    }

    @Override
    public void setTUnitNumber(String tUnitNumber) {
        try {
            super.setTUnitNumber(tUnitNumber);
            String unit = tUnitNumber.replaceFirst("-?\\d+((\\.|/)\\d+)?", "");
            String number = tUnitNumber.substring(0,
                    tUnitNumber.length() - unit.length());
            unit = unit.trim();
            this.number = Rationals.create(number);
            if (!unit.isEmpty())
                this.unit = Unit.valueOf(unit);
        } catch(Throwable t) {
            System.out.println(t);
        }
    }

    @Override
    public void setTUnitInf(String tUnitInf) {
        try {
            tUnitInf = tUnitInf.trim();
            super.setTUnitInf(tUnitInf);
            String unit;
            if (tUnitInf.startsWith("oo")) {
                this.infSign = true;
                unit = tUnitInf.substring(3,tUnitInf.length());
            } else {
                if(tUnitInf.startsWith("+oo")) {
                    this.infSign = true;
                } else {
                    this.infSign = false;
                }
                unit = tUnitInf.substring(4,tUnitInf.length());
            }
            this.unit = Unit.valueOf(unit);
        } catch(Throwable t) {
            System.out.println(t);
        }
    }

    public Optional<Rational> getNumber() {
        return tUnitNumber.isPresent() ? Optional.of(number) : Optional.empty() ;
    }

    public Optional<Boolean> getInfSign() {
        return tUnitInf.isPresent() ? Optional.of(infSign) : Optional.empty() ;
    }

    public Optional<Unit> getUnit() {
        return (tUnitInf.isPresent() || tUnitNumber.isPresent()) ? Optional.of(unit) : Optional.empty() ;
    }

    public static ASTUnitNumber valueOf(Rational number, Unit unit) {
        return new ASTUnitNumber(number, unit);
    }

    @Override
    public String toString() {
        String inf;
        if(infSign) {inf = "+oo";} else {inf = "-oo";}
        return tUnitNumber.isPresent() ? String.format("%s %s", number, unit) : String.format("%s %s", inf, unit);
    }

    @Override
    public int hashCode() {
        return Amount.valueOf(number.doubleValue(), unit).hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this)
            return true;

        if (!(obj instanceof ASTUnitNumber))
            return false;

        ASTUnitNumber un = (ASTUnitNumber)obj;
        if (!(un.unit.isCompatible(this.unit)))
            return false;

        // TODO make rational comparison
        return Amount.valueOf(number.doubleValue(), unit).equals(
                Amount.valueOf(un.number.doubleValue(), un.unit));
    }

    public void setUnit(Unit unit) { this.unit = unit; }
    public void setNumber(Rational number) { this.number = number; }
    public void setInfSign(Boolean infSign) { this.infSign = infSign; }

    public  ASTUnitNumber deepClone()   {

        return deepClone(_construct());

    }


    public  ASTUnitNumber deepClone(ASTUnitNumber result)   {
        Log.errorIfNull(result, "0xA7006_285 Parameter 'result' must not be null.");

        super.deepClone(result);

        result.setInfSign(this.getInfSign().orElse(true));
        result.setNumber(this.getNumber().orElse(Rational.ZERO));
        result.setUnit(this.getUnit().orElse(Unit.ONE));

        return result;

    }
}
