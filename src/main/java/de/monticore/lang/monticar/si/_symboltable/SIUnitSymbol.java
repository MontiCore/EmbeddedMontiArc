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
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import org.jscience.mathematics.number.Rational;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;

import javax.measure.unit.Unit;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class SIUnitSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final SIUnitKind KIND = SIUnitKind.INSTANCE;

    Unit unit;
    Rational number;

    protected SIUnitSymbol(String name, ASTUnitNumber astUnitNumber) {
        super(name, KIND);
        this.unit = astUnitNumber.getUnit().orElse(null);
        this.number = astUnitNumber.getNumber().orElse(null);
    }

    public Unit getUnit() {
        return this.unit;
    }

    public void setUnit(Unit unit) {
        this.unit = unit;
    }

    public Rational getNumber() {
        return this.number;
    }

    public void setNumber(Rational number) {
        this.number = number;
    }

    @Override
    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return null;
    }

    @Override
    public Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass() {
        return Optional.empty();
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces() {
        return Collections.emptyList();
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes() {
        return Collections.emptyList();
    }

    @Override
    public boolean isFormalTypeParameter() {
        return false;
    }

    @Override
    public Scope getSpannedScope() {
        return null;
    }
}
