/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.symboltable.SymbolKind;

public class MontiCarFieldSymbol extends CommonMCFieldSymbol<MontiCarTypeSymbolReference> {

    public static final MontiCarFieldSymbolKind KIND = new MontiCarFieldSymbolKind();

    public MontiCarFieldSymbol(String name, MCAttributeSymbolKind kind, MontiCarTypeSymbolReference type) {
        super(name, kind);
        setType(type);
    }

    public static class MontiCarFieldSymbolKind extends MCAttributeSymbolKind {

        private static final String NAME = MontiCarFieldSymbolKind.class.getCanonicalName();

        protected MontiCarFieldSymbolKind() {
        }

        @Override
        public String getName() {
            return NAME;
        }

        @Override
        public boolean isKindOf(SymbolKind kind) {
            return NAME.equals(kind.getName()) || super.isKindOf(kind);
        }
    }
}
