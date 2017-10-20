/**
 * ******************************************************************************
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
