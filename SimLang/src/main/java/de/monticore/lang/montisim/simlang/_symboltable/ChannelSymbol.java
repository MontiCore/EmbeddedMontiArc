package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.Channel;

public class ChannelSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {
  private Channel channel;

  public static final ChannelKind KIND = ChannelKind.INSTANCE;

  public ChannelSymbol(String name, Channel channel) {
    super(name, KIND);
    this.channel = channel;
  }

  public Channel getChannel() {
    return channel;
  }
}
