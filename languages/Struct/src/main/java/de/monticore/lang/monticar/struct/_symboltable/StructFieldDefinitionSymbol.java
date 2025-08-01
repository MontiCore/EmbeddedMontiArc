/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.lang.monticar.ts.CommonMCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

public class StructFieldDefinitionSymbol extends CommonMCFieldSymbol<MCTypeReference<? extends MCTypeSymbol>> {

    public StructFieldDefinitionSymbol(String name) {
        super(name);
    }
}
