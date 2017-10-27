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
package de.monticore.lang.monticar.struct.coco;

import de.monticore.lang.monticar.types2._ast.ASTPrimitiveArrayType;
import de.monticore.lang.monticar.types2._cocos.Types2ASTPrimitiveArrayTypeCoCo;
import de.se_rwth.commons.logging.Log;

public class PrimitiveArrayTypesAreForbidden implements Types2ASTPrimitiveArrayTypeCoCo {
    @Override
    public void check(ASTPrimitiveArrayType node) {
        Log.error(
                "Primitive array types are forbidden",
                node.get_SourcePositionStart()
        );
    }
}
