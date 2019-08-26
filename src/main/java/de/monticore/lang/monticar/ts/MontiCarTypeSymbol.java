/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.symboltable.SymbolKind;

public class MontiCarTypeSymbol extends CommonMCTypeSymbol<MontiCarTypeSymbol, MontiCarTypeSymbolReference> {

    public static final MontiCarTypeSymbolKind KIND = new MontiCarTypeSymbolKind();

    public MontiCarTypeSymbol(String name) {
        this(name, MontiCarTypeSymbol.KIND);
    }

    protected MontiCarTypeSymbol(String name, MCTypeSymbolKind typeKind) {
        super(name, typeKind);
    }

    public static class MontiCarTypeSymbolKind extends MCTypeSymbolKind {

        private static final String NAME = MontiCarTypeSymbolKind.class.getCanonicalName();

        protected MontiCarTypeSymbolKind() {
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
