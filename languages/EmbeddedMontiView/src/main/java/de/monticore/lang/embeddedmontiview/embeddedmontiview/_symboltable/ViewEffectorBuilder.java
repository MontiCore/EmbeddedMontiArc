/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class ViewEffectorBuilder {
  protected Optional<String> source = Optional.empty();
  protected Optional<String> target = Optional.empty();
  //  protected Optional<ConstantPortSymbol> portSymbol = Optional.empty();

  public static ViewEffectorSymbol clone(ViewEffectorSymbol con) {
    return new ViewEffectorBuilder().setSource(con.getSource()).
        setTarget(con.getTarget()).build();
  }

  public ViewEffectorBuilder setSource(String source) {
    this.source = Optional.of(source);
    return this;
  }

  public ViewEffectorBuilder setTarget(String target) {
    this.target = Optional.of(target);
    return this;
  }

  public ViewEffectorSymbol build() {
    if (source.isPresent() && target.isPresent()) {
      ViewEffectorSymbol con = new ViewEffectorSymbol(this.target.get());
      con.setSource(this.source.get());
      con.setTarget(this.target.get());
      return con;
    }

    Log.error("not all parameters have been set before to build the effector symbol");
    throw new Error("not all parameters have been set before to build the effector symbol");
  }
}
