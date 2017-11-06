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

import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;

/**
 * TODO: Write me!
 *
 * @author (last commit) $Author$
 * @version $Revision$, $Date$
 * @since TODO: add version number
 */
public class MontiCarSymbolFactory implements MCTypeFactory<MontiCarTypeSymbol> {

    // ----------------------
    // attributes
    // ----------------------

    public MontiCarFieldSymbol createFormalParameterSymbol(String name, MontiCarTypeSymbolReference type) {
        MontiCarFieldSymbol formalParameterSymbol = new MontiCarFieldSymbol(name,
                MontiCarFieldSymbol.KIND, type);

        // init
        formalParameterSymbol.setParameter(true);

        return formalParameterSymbol;
    }

    public MontiCarFieldSymbol createFieldSymbol(String name, MontiCarTypeSymbolReference typeSymbol) {
        MontiCarFieldSymbol fieldSymbol = new MontiCarFieldSymbol(name, MontiCarFieldSymbol.KIND,
                typeSymbol);

        // init (no init yet)

        return fieldSymbol;
    }

    public MontiCarFieldSymbol createLocalVariableSymbol(String name,
                                                         MontiCarTypeSymbolReference typeSymbol) {
        MontiCarFieldSymbol localVariableSymbol = new MontiCarFieldSymbol(name,
                MontiCarFieldSymbol.KIND, typeSymbol);

        // init (no init yet)

        return localVariableSymbol;
    }

    // ----------------------
    // methods
    // ----------------------
    public MontiCarMethodSymbol createMethod(String name) {
        MontiCarMethodSymbol methodSymbol = new MontiCarMethodSymbol(name, MontiCarMethodSymbol.KIND);
        return methodSymbol;
    }

    public MontiCarMethodSymbol createConstructor(String name) {
        MontiCarMethodSymbol constructorSymbol = new MontiCarMethodSymbol(name, MontiCarMethodSymbol.KIND);

        // init
        constructorSymbol.setConstructor(true);

        return constructorSymbol;
    }

    // ----------------------
    // types
    // ----------------------
    public MontiCarTypeSymbol createClassSymbol(String name) {
        MontiCarTypeSymbol typeVariableSymbol = new MontiCarTypeSymbol(name);

        // class init (nothing here yet)

        return typeVariableSymbol;
    }

    public MontiCarTypeSymbol createInterfaceSymbol(String name) {
        MontiCarTypeSymbol interfaceSymbol = new MontiCarTypeSymbol(name);

        // interface init
        interfaceSymbol.setInterface(true);
        interfaceSymbol.setAbstract(true);

        return interfaceSymbol;
    }

    @Override
    public MontiCarTypeSymbol createTypeVariable(String name) {
        MontiCarTypeSymbol typeVariableSymbol = new MontiCarTypeSymbol(name);

        // type variable init
        // TODO do these serve the same purpose? if yes type variable is redundant
        typeVariableSymbol.setFormalTypeParameter(true);
        typeVariableSymbol.setTypeVariable(true);

        return typeVariableSymbol;
    }

    public MontiCarTypeSymbol createEnumSymbol(String name) {
        MontiCarTypeSymbol enumTypeSymbol = new MontiCarTypeSymbol(name);

        // enum init
        enumTypeSymbol.setEnum(true);

        return enumTypeSymbol;
    }

    public MontiCarTypeSymbol createAnnotation(String name) {
        MontiCarTypeSymbol annotationTypeSymbol = new MontiCarTypeSymbol(name);

        // Annotation init
        annotationTypeSymbol.setAnnotation(true);

        return annotationTypeSymbol;
    }
}
