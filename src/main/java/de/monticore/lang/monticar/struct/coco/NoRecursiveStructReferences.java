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

import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._cocos.StructASTStructCoCo;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class NoRecursiveStructReferences implements StructASTStructCoCo {

    private final static String NEW_LINE = System.lineSeparator();

    private final Set<String> visitedStructs = new HashSet<>();

    @Override
    public void check(ASTStruct node) {
        if (node.getSymbol().isPresent()) {
            checkStruct((StructSymbol) node.getSymbol().get());
        }
    }

    private void checkStruct(StructSymbol symbol) {
        String structFullName = symbol.getFullName();
        boolean isCycle = !visitedStructs.add(structFullName);
        if (isCycle) {
            logCycle(symbol);
            return;
        }
        for (StructFieldDefinitionSymbol f : symbol.getStructFieldDefinitions()) {
            MCTypeSymbol s = f.getType().getReferencedSymbol();
            if (s instanceof StructSymbol) {
                checkStruct((StructSymbol) s);
            }
        }
        visitedStructs.remove(structFullName);
    }

    private void logCycle(StructSymbol symbol) {
        StringBuilder message = new StringBuilder("The following structures form a cycle by referencing:");
        for (String s : visitedStructs) {
            message.append(NEW_LINE);
            message.append(s);
        }
        Log.error(message.toString(), symbol.getSourcePosition());
    }
}
