/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp.util;

import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.SourcePosition;

class SymbolRange {
    private final SourcePosition start;
    private final SourcePosition end;
    private final Symbol symbol;

    public SymbolRange(SourcePosition start, SourcePosition end, Symbol symbol) {
        this.start = start;
        this.end = end;
        this.symbol = symbol;
    }

    boolean contains(SourcePosition position){
        return start.compareTo(position) <= 0 && end.compareTo(position) >= 0;
    }

    public SourcePosition getStart() {
        return start;
    }

    public SourcePosition getEnd() {
        return end;
    }

    public Symbol getSymbol() {
        return symbol;
    }
}
