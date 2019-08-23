/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import java.util.Optional;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

public class EMAPortBuilder {
  protected Optional<Boolean> incoming = Optional.empty();
  protected Optional<String> name = Optional.empty();
  protected Optional<MCTypeReference<? extends MCTypeSymbol>> typeReference = Optional.empty();

  public static ViewPortSymbol clone(ViewPortSymbol port) {
    return new EMAPortBuilder().setName(port.getName()).setDirection(port.isIncoming()).setTypeReference(port.getTypeReference()).build();
  }

  public EMAPortBuilder setDirection(boolean incoming) {
    this.incoming = Optional.of(Boolean.valueOf(incoming));
    return this;
  }

  public EMAPortBuilder setName(String name) {
    this.name = Optional.of(name);
    return this;
  }

  public EMAPortBuilder setTypeReference(Optional<MCTypeReference<? extends MCTypeSymbol>> typeReference) {
    this.typeReference = typeReference;
    return this;
  }

  public ViewPortSymbol build() {
    if (name.isPresent() && incoming.isPresent()) {
      ViewPortSymbol p = new ViewPortSymbol(this.name.get());
      p.setDirection(this.incoming.get());
      p.setTypeReference(this.typeReference);
      return p;
    }
    Log.error("not all parameters have been set before to build the port symbol");
    throw new Error("not all parameters have been set before to build the port symbol");
  }

  //    public ConstantPortSymbol buildConstantPort(MCTypeReference typeReference) {
  //        if (typeReference == null) {
  //            Log.error("not all parameters have been set before to build the port symbol");
  //            throw new Error("not all parameters have been set before to build the port symbol");
  //        }
  //        ConstantPortSymbol p = new ConstantPortSymbol();
  //        p.setDirection(false);
  //        p.setTypeReference(typeReference);
  //        return p;
  //    }
}
