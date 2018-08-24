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

import de.monticore.lang.monticar.ranges._ast.ASTRange;

import java.util.Optional;

/**
 * @author Christoph Richter
 */
public class ASTElementType extends ASTElementTypeTOP {

    public ASTElementType() {
    }

    public ASTElementType(Optional<String> name, Optional<ASTRange> range) {
        super(name, range);
    }

    public boolean isBoolean() {
        return isPresentName() &&  getName().contentEquals("B") || getName().contentEquals("Boolean");
    }

    public boolean isNaturalNumber() {
        return isPresentName() && getName().contentEquals("N");
    }

    public boolean isWholeNumber() {
        return isPresentName() && getName().contentEquals("Z");
    }

    public boolean isRational() {
        return isPresentName() && getName().contentEquals("Q");
    }

    public boolean isComplex() {
        return isPresentName() && getName().contentEquals("C");
    }

    @Override
    public String getName() {
        if (!super.isPresentName())
            return "Q";
        return super.getName();
    }

}
