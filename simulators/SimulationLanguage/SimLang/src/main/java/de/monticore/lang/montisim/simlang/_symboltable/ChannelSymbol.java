/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.Channel;

public class ChannelSymbol extends de.monticore.symboltable.CommonScopeSpanningSymbol {
  private Channel channel;

  public static final ChannelKind KIND = ChannelKind.INSTANCE;

  public ChannelSymbol(String name) {
    super(name, KIND);
  }
  public ChannelSymbol(String name, Channel channel) {
    super(name, KIND);
    this.channel = channel;
  }
  public void setChannel(Channel channel) { this.channel = channel; }
  public Channel getChannel() {
    return channel;
  }
}
