package de.monticore.lang.montisim.simlang._symboltable;

import java.util.Collection;
import java.util.Optional;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

public class ChannelSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {

  public static final ChannelSymbolKind KIND = ChannelSymbolKind.INSTANCE;

  public ChannelSymbol(String name) {
    super(name, KIND);
  }
}
