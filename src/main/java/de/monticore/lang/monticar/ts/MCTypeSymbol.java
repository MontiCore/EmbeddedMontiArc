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
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.monticore.symboltable.types.TypeSymbol;

import java.util.List;
import java.util.Optional;

/**
 * @author Pedram Mir Seyed Nazari
 */
public interface MCTypeSymbol extends TypeSymbol, ScopeSpanningSymbol {

    MCTypeSymbolKind KIND = new MCTypeSymbolKind();

    boolean isGeneric();

    List<? extends MCTypeSymbol> getFormalTypeParameters();

    Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass();

    List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces();

    List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes();

    List<? extends MCFieldSymbol> getFields();

    Optional<? extends MCFieldSymbol> getField(String attributeName);

    List<? extends MCMethodSymbol> getMethods();

    Optional<? extends MCMethodSymbol> getMethod(String methodName);

    List<? extends MCMethodSymbol> getConstructors();

    List<? extends MCTypeSymbol> getInnerTypes();

    Optional<? extends MCTypeSymbol> getInnerType(String innerTypeName);

    /**
     * @return true, if type is an abstract class or an interface
     */
    boolean isAbstract();

    boolean isFinal();

    boolean isInterface();

    boolean isEnum();

    boolean isClass();

    /**
     * @return true, if this type is an inner type, such as an inner interface or inner class
     */
    boolean isInnerType();

    boolean isPrivate();

    boolean isProtected();

    boolean isPublic();

    /**
     * @return true, if this type itself is a formal type parameter.
     */
    boolean isFormalTypeParameter();
}
