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
import de.monticore.lang.monticar.ranges._parser.RangesParser;

import java.io.IOException;
import java.util.Optional;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class ASTElementType extends ASTElementTypeTOP {
    protected Optional<ASTRange> range = Optional.empty();
    protected static RangesParser rangesParser = new RangesParser();

    public  ASTElementType (String tElementType,
        boolean isBoolean, boolean isComplex, boolean isRational,
                               boolean isWholeNumber, boolean isNaturalNumber) {
        super(tElementType, isBoolean, isComplex, isRational, isWholeNumber, isNaturalNumber);
    }

    public ASTElementType() {
        super();
    }

    public Optional<ASTRange> getRange() {
        return range;
    }

    /**
     token TElementType =
      ('Z' | 'Q' | 'C') Space*
        '(' Space* (TUnitNumber | TUnitInf) Space* ':'
            (Space* TUnitNumber Space* ':')?
            Space*
    */
    @Override
    public void setTElementType(String tElementType) {
        super.setTElementType(tElementType);

        isWholeNumberNumber = tElementType.startsWith("Z");
        isRational = tElementType.startsWith("Q");
        isComplex = tElementType.startsWith("C");
        isBoolean = false;

        String r = tElementType.substring(1).trim();
        try {
            range = rangesParser.parseString_Range(r);
        } catch (IOException e) {
            e.printStackTrace(); // no error should occur since it is a string

        }
    }
}
