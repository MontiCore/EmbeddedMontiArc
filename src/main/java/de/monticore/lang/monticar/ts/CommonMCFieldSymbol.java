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
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;

/**
 * @author Pedram Mir Seyed Nazari
 */
public abstract class CommonMCFieldSymbol<T extends MCTypeReference<? extends MCTypeSymbol>> extends CommonSymbol implements MCFieldSymbol {

    private T type;

    private boolean isParameter = false;

    public CommonMCFieldSymbol(String name) {
        this(name, MCFieldSymbol.KIND);
    }

    public CommonMCFieldSymbol(String name, MCAttributeSymbolKind kind) {
        super(name, kind);
    }

    @Override
    public T getType() {
        return type;
    }

    public void setType(T type) {
        this.type = type;
    }

    public void setParameter(boolean isParameter) {
        this.isParameter = isParameter;
    }

    @Override
    public boolean isParameter() {
        return isParameter;
    }
}
