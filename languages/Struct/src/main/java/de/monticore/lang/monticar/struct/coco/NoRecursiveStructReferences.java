/* (c) https://github.com/MontiCore/monticore */
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
        if (node.getSymbolOpt().isPresent()) {
            checkStruct((StructSymbol) node.getSymbolOpt().get());
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
