package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.Channel;

import java.util.Collection;
import java.util.Optional;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

public class ChannelSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {
  private Channel channel;

  public static final ChannelSymbolKind KIND = ChannelSymbolKind.INSTANCE;

  public ChannelSymbol(String name, Channel channel) {
    super(name, KIND);
    this.channel = channel;
  }

  public Channel getChannel() {
    return channel;
  }
}
