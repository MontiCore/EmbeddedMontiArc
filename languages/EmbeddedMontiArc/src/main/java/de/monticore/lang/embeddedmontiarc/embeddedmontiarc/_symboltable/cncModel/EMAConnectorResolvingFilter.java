/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingInfo;

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
