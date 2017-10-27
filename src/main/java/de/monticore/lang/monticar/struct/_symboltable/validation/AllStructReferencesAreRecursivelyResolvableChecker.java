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
package de.monticore.lang.monticar.struct._symboltable.validation;

import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.struct.model.type.StructFieldTypeInfo;
import de.monticore.lang.monticar.struct.model.type.StructReferenceFieldType;
import de.monticore.lang.monticar.struct.model.type.VectorStructFieldType;
import de.monticore.symboltable.references.FailedLoadingSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class AllStructReferencesAreRecursivelyResolvableChecker implements SymbolChecker<StructSymbol> {

    private final static String NEW_LINE = System.lineSeparator();

    private final Set<String> visitedStructs = new HashSet<>();

    @Override
    public boolean isValid(StructSymbol symbol) {
        String structFullName = symbol.getFullName();
        boolean isCycle = visitedStructs.contains(structFullName);
        if (isCycle) {
            StringBuilder message = new StringBuilder("the following structures form a cycle by referencing:");
            for (String s : visitedStructs) {
                message.append(NEW_LINE);
                message.append(s);
            }
            Log.error(message.toString(), symbol.getSourcePosition());
            return false;
        }
        visitedStructs.add(structFullName);
        Collection<StructFieldDefinitionSymbol> fieldDefinitions = symbol.getStructFieldDefinitions();
        for (StructFieldDefinitionSymbol f : fieldDefinitions) {
            StructFieldTypeInfo fieldTypeInfo = f.getTypeInfo();
            if (fieldTypeInfo instanceof StructReferenceFieldType) {
                if (!checkReferenceFieldType((StructReferenceFieldType) fieldTypeInfo)) {
                    return false;
                }
            } else if (fieldTypeInfo instanceof VectorStructFieldType) {
                StructFieldTypeInfo typeOfElements = ((VectorStructFieldType) fieldTypeInfo).getTypeOfElements();
                if (typeOfElements instanceof StructReferenceFieldType) {
                    if (!checkReferenceFieldType((StructReferenceFieldType) typeOfElements)) {
                        return false;
                    }
                }
            }
        }
        visitedStructs.remove(structFullName);
        return true;
    }

    private boolean checkReferenceFieldType(StructReferenceFieldType referenceFieldType) {
        StructSymbol referencedStruct;
        try {
            referencedStruct = referenceFieldType.getReference().getReferencedSymbol();
        } catch (FailedLoadingSymbol e) {
            Log.error(e.getMessage());
            return false;
        }
        return referencedStruct != null && isValid(referencedStruct);
    }
}
