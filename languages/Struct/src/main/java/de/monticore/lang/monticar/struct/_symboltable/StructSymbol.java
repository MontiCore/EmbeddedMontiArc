/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.lang.monticar.ts.CommonMCTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbolKind;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.Symbols;

import java.util.Collection;

public class StructSymbol extends CommonMCTypeSymbol<MCTypeSymbol, MCTypeReference<MCTypeSymbol>> implements MCTypeSymbol {

    public static final MCTypeSymbolKind KIND = MCTypeSymbol.KIND;

    public StructSymbol(String name) {
        super(name, KIND);
    }

    public Collection<StructFieldDefinitionSymbol> getStructFieldDefinitions() {
        return Symbols.sortSymbolsByPosition(getSpannedScope().resolveLocally(StructFieldDefinitionSymbol.KIND));
    }
}
