/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingInfo;

import java.util.*;

/**
 * Created by MichaelvonWenckstern on 02.09.2016.
 */
public class EMAConnectorResolvingFilter<S extends Symbol>
    extends CommonResolvingFilter<S> {

  public EMAConnectorResolvingFilter(SymbolKind targetKind) {
    super(targetKind);
  }

  @Override
  public Optional<Symbol> filter(ResolvingInfo resolvingInfo, String name, Map<String, Collection<Symbol>> symbols) {
    final Set<Symbol> resolvedSymbols = new LinkedHashSet<>();

    final Collection<Symbol> allSymbols = getSymbolsAsCollection(symbols);

    for (Symbol symbol : allSymbols) {
      if (symbol.isKindOf(getTargetKind())) {

        if (symbol.getName().equals(name) || symbol.getFullName().equals(name)) {
          resolvedSymbols.add(symbol);
        }
      }
    }

    return ResolvingFilter.getResolvedOrThrowException(resolvedSymbols);
  }

  private Collection<Symbol> getSymbolsAsCollection(Map<String, Collection<Symbol>> symbols) {
    final Collection<Symbol> r = new LinkedHashSet<>();
    symbols.values().forEach(r::addAll);
    return r;
  }
}
