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
package de.monticore.lang.monticar.types2._ast;

import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.List;
import java.util.Optional;

import de.monticore.lang.numberunit._ast.ASTUnitNumber;

/**
 * Created by Sascha on 01.06.2017.
 */
public class ASTUnitNumberResolution extends ASTUnitNumberResolutionTOP {
    public ASTUnitNumberResolution() {
        super();
    }

    public ASTUnitNumberResolution(Optional<String> name, Optional<ASTUnitNumber> unitNumber) {
        super(name, unitNumber);
    }

    public void setNumber(Rational number) {
        if (!getUnitNumberOpt().isPresent()) {
            setUnitNumber(new de.monticore.lang.numberunit._ast.ASTUnitNumber(number, Unit.ONE));
        }
        getUnitNumberOpt().get().setNumber(number);
    }

    public void setUnit(Unit unit) {
        if (!getUnitNumberOpt().isPresent()) {
            setUnitNumber(new de.monticore.lang.numberunit._ast.ASTUnitNumber(Rational.ZERO, unit));
        }
        getUnitNumberOpt().get().setUnit(unit);
    }

    public Optional<Rational> getNumber() {
        return getUnitNumberOpt().get().getNumber();
    }

    public Optional<Unit> getUnit() {
        return getUnitNumberOpt().get().getUnit();
    }

    public String doResolution(List<ResolutionDeclarationSymbol> resolutionDeclarationSymbolList) {
        if (getNameOpt().isPresent()) {
            for (ResolutionDeclarationSymbol resDeclSym : resolutionDeclarationSymbolList) {

                if (resDeclSym.getNameToResolve().equals(getNameOpt().get())) {

                    Log.debug(resDeclSym.getASTResolution().toString(), "Found ResolutionDeclarationSymbol:");
                    ASTUnitNumber toSet = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getUnitNumberOpt().get();


                    Log.debug("" + toSet.getNumber().get().intValue(), "ToSet Number:");

                    setNumber(toSet.getNumber().get());
                    setUnit(toSet.getUnit().get());
                    Log.debug("" + getNumber().get().intValue(), "PortResolution Number:");
                    Log.debug(getNameOpt().get(),"Name:");
                    return getNameOpt().get();
                }
            }

        }
        return null;
    }

    @Override
    public String printType(){
        return "UnitNumberResolution";
    }
}
