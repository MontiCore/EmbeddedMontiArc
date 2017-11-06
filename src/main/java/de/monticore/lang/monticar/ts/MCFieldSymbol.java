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
package de.monticore.lang.monticar.ts;/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */

import de.monticore.symboltable.Symbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

/**
 * @author Pedram Mir Seyed Nazari
 *
 */
public interface MCFieldSymbol extends Symbol {

    MCAttributeSymbolKind KIND = new MCAttributeSymbolKind();

    MCTypeReference<? extends MCTypeSymbol> getType();

    boolean isStatic();

    boolean isFinal();

    boolean isParameter();

    boolean isPrivate();

    boolean isProtected();

    boolean isPublic();
}
