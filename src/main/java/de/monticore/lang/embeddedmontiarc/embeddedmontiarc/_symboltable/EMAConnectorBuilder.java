/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 */
public class EMAConnectorBuilder {
  protected Optional<String> source = Optional.empty();
  protected Optional<String> target = Optional.empty();
  protected Optional<EMAConstantPortSymbol> portSymbol = Optional.empty();

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
 
  public EMAConnectorBuilder setConstantPortSymbol(EMAConstantPortSymbol portSymbol) {
    this.portSymbol = Optional.of(portSymbol);
    return this;
  }

  public EMAConnectorSymbol build() {
    if (source.isPresent() && target.isPresent()) {
      EMAConnectorSymbol con = new EMAConnectorSymbol(this.target.get());
      con.setSource(this.source.get());
      con.setTarget(this.target.get());
	  if(portSymbol.orElse(null) != null) {
        con.setEMAConstantPortSymbol(portSymbol.get());
	  }
      return con;
    }
    Log.error("not all parameters have been set before to build the connector symbol");
    throw new Error("not all parameters have been set before to build the connector symbol");
  }
}
