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
package de.monticore.lang.monticar.emadl._ast;

import de.monticore.lang.monticar.common2._ast.ASTLiteralValue;
import de.monticore.lang.monticar.common2._ast.ASTValue;
import de.monticore.lang.monticar.literals2._ast.ASTBooleanLiteral;
import de.monticore.lang.monticar.literals2._ast.ASTCharLiteral;
import de.monticore.lang.monticar.literals2._ast.ASTStringLiteral;
import de.monticore.lang.numberunit._ast.ASTNumber;
import org.jscience.mathematics.number.Rational;

import java.util.Optional;

public class ASTNamedArgument extends ASTNamedArgumentTOP {

    public ASTNamedArgument() {
    }

    public ASTNamedArgument(String name, ASTValue value) {
        super(name, value);
    }
    
    public Object getLiteralValue() {
        if (getValue() instanceof ASTLiteralValue){
            return ((ASTLiteralValue) getValue()).getValue();
        }
        return super.getValue();
    }

    public Optional<Rational> getRationalValue(){
        if (getLiteralValue() instanceof ASTNum){
            ASTNumber number = ((ASTNum) getLiteralValue()).getNumber();
            return number.getUnitNumber().get().getNumber();
        }
        return Optional.empty();
    }

    public Optional<Integer> getIntValue(){
        Optional<Rational> rational = getRationalValue();
        if (rational.isPresent() && rational.get().getDivisor().intValue() == 1){
            return Optional.of(rational.get().getDividend().intValue());
        }
        return Optional.empty();
    }

    public Optional<Double> getDoubleValue(){
        return getRationalValue().map(Rational::doubleValue);
    }

    public Optional<Boolean> getBooleanValue(){
        if (getLiteralValue() instanceof ASTBooleanLiteral){
            return Optional.of(((ASTBooleanLiteral) getLiteralValue()).getValue());
        }
        return Optional.empty();
    }

    public Optional<String> getStringValue(){
        if (getLiteralValue() instanceof ASTStringLiteral){
            return Optional.of(((ASTStringLiteral) getLiteralValue()).getValue());
        }
        else if (getLiteralValue() instanceof ASTCharLiteral){
            return Optional.of(String.valueOf(((ASTCharLiteral) getLiteralValue()).getValue()));
        }
        return Optional.empty();
    }
}
