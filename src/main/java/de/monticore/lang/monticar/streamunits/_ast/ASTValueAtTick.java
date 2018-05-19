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
package de.monticore.lang.monticar.streamunits._ast;

import de.monticore.lang.numberunit._ast.ASTUnitNumber;

import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class ASTValueAtTick extends ASTValueAtTickTOP {
    public ASTValueAtTick() {
    }

    public ASTValueAtTick(String name, Optional<ASTUnitNumber> value, Optional<ASTUnitNumber> lowerBound, Optional<ASTUnitNumber> upperBound) {
        super(name, value, lowerBound, upperBound);
    }

    @Override
    public String toString() {
        String result = "";
        result += name + "(";
        if (getValueOpt().isPresent()) {
            result += value.get().getNumber().get().intValue();
        } else {
            if (getLowerBoundOpt().isPresent()) {
                result += lowerBound.get().getNumber().get().intValue();
            }
            if (getUpperBoundOpt().isPresent()) {
                result += ":" + upperBound.get().getNumber().get().intValue();
            }
        }
        result += ")";
        return result;
    }
}
