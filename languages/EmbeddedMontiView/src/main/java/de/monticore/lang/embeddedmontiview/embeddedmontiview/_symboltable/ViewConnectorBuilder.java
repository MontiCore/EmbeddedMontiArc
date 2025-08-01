/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 */
public class ViewConnectorBuilder {
  protected Optional<String> source = Optional.empty();
  protected Optional<String> target = Optional.empty();
  //  protected Optional<ConstantPortSymbol> portSymbol = Optional.empty();

  public static ViewConnectorSymbol clone(ViewConnectorSymbol con) {
    return new ViewConnectorBuilder().setSource(con.getSource()).
        setTarget(con.getTarget()).build();
  }

  public ViewConnectorBuilder setSource(String source) {
    this.source = Optional.of(source);
    return this;
  }

  public ViewConnectorBuilder setTarget(String target) {
    this.target = Optional.of(target);
    return this;
  }

  //  public ViewConnectorBuilder setConstantPortSymbol(ConstantPortSymbol portSymbol) {
  //    this.portSymbol = Optional.of(portSymbol);
  //    return this;
  //  }

  public ViewConnectorSymbol build() {
    if (source.isPresent() && target.isPresent()) {
      ViewConnectorSymbol con = new ViewConnectorSymbol(this.target.get());
      con.setSource(this.source.get());
      con.setTarget(this.target.get());
      //	  if(portSymbol.orElse(null) != null) {
      //        con.setConstantPortSymbol(portSymbol.get());
      //	  }
      return con;
    }
    Log.error("not all parameters have been set before to build the connector symbol");
    throw new Error("not all parameters have been set before to build the connector symbol");
  }
}
