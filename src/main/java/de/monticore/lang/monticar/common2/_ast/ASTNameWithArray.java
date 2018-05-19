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
package de.monticore.lang.monticar.common2._ast;

import java.util.Optional;

/**
 * Created by Sascha on 04.07.2017.
 */
public class ASTNameWithArray extends ASTNameWithArrayTOP {

    protected ASTNameWithArray() {
        super();
    }

    protected ASTNameWithArray(String name, Optional<ASTArrayDeclaration> arrayDeclaration) {
        super(name, arrayDeclaration);
    }

    @Override
    public String getName() {
        if (getArrayDeclarationOpt().isPresent()) {
            return name + "[" + getArrayDeclarationOpt().get().getIntLiteral().toString() + "]";
        }
        return name;
    }
}
