package de.monticore.lang.monticar.generator.roscpp.cocos;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.ScopeSpanningSymbol;
import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class EmbeddedMontiArcMathSymtabCoCoChecker {
    private List<EmbeddedMontiArcMathSymtabCoCo> coCos = new ArrayList<>();
    private TaggingResolver taggingResolver;

    public EmbeddedMontiArcMathSymtabCoCoChecker(TaggingResolver taggingResolver) {
        this.taggingResolver = taggingResolver;
    }

    public EmbeddedMontiArcMathSymtabCoCoChecker addCoCo(EmbeddedMontiArcMathSymtabCoCo coCo) {
        coCos.add(coCo);
        return this;
    }

    public void checkAll(Symbol symbol) {
        coCos.forEach(coCo -> coCo.check(taggingResolver, symbol));

        if (symbol instanceof ScopeSpanningSymbol) {
            ((ScopeSpanningSymbol) symbol)
                    .getSpannedScope()
                    .getLocalSymbols()
                    .values().stream()
                    .flatMap(Collection::stream)
                    .forEach(this::checkAll);
        }
    }

}
