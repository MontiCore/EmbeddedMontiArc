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
package de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagDelaySymbol extends TagSymbol {
  public static final TagDelaySymbolKind KIND = TagDelaySymbolKind.INSTANCE;

  public TagDelaySymbol(Integer delayLength) {
    super(KIND, delayLength);
  }

  @Override
  public String toString() {
    return String.format("TagDelayLength = %s", getDelayLength());
  }

  public Integer getDelayLength() {
    return getValue(0);
  }

  public static class TagDelaySymbolKind extends TagKind {

    public static final TagDelaySymbolKind INSTANCE = new TagDelaySymbolKind();

    protected TagDelaySymbolKind() {}
  }
}

