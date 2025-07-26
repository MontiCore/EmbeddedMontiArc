/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 */
public class EMAConnectorBuilder {
  protected Optional<String> source = Optional.empty();
  protected Optional<String> target = Optional.empty();
  protected Optional<EMAPortSymbol> portSymbol = Optional.empty();

  public static EMAConnectorSymbol clone(EMAConnectorSymbol con) {
    return new EMAConnectorBuilder().setSource(con.getSource()).
        setTarget(con.getTarget()).build();
  }

  public EMAConnectorBuilder setSource(String source) {
    this.source = Optional.of(source);
    return this;
  }

  public EMAConnectorBuilder setTarget(String target) {
    this.target = Optional.of(target);
    return this;
  }
 
  public EMAConnectorBuilder setConstantPortSymbol(EMAPortSymbol portSymbol) {
    this.portSymbol = Optional.of(portSymbol);
    return this;
  }

  public EMAConnectorSymbol build() {
    if (source.isPresent() && target.isPresent()) {
      EMAConnectorSymbol con = new EMAConnectorSymbol(this.target.get());
      con.setSource(this.source.get());
      con.setTarget(this.target.get());
	  if(portSymbol.orElse(null) != null) {
        con.setConstantEMAPortSymbol(portSymbol.get());
	  }
      return con;
    }
    Log.error("not all parameters have been set before to build the connector symbol");
    throw new Error("not all parameters have been set before to build the connector symbol");
  }

  public static EMAConnectorInstanceSymbol instantiate(EMAConnectorSymbol connector, String packageName) {
    EMAConnectorInstanceSymbol connectorInstance = new EMAConnectorInstanceSymbol(connector.getTarget());
    connectorInstance.setSource(connector.getSource());
    connectorInstance.setTarget(connector.getTarget());
    connectorInstance.setIsConstantConnector(connector.isConstant);
    connectorInstance.setConstantEMAPortSymbol(connector.constantEmaPortSymbol);
    connectorInstance.setPackageName(packageName);
    connectorInstance.setFullName(packageName + "." + connectorInstance.getName());
    return connectorInstance;
  }
}
