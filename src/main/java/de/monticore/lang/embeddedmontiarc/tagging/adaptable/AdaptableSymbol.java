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
/* generated by template templates.de.monticore.lang.tagschema.SimpleTagType*/


package de.monticore.lang.embeddedmontiarc.tagging.adaptable;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by SimpleTagType.ftl
 */
public class AdaptableSymbol extends TagSymbol {
  public static final AdaptableKind KIND = AdaptableKind.INSTANCE;

  /**
   * is marker symbol so it has no value by itself
   */
  public AdaptableSymbol() {
    super(KIND);
  }

  protected AdaptableSymbol(AdaptableKind kind) {
    super(kind);
  }

  @Override
  public String toString() {
    return "Adaptable";
  }

  public static class AdaptableKind extends TagKind {
    public static final AdaptableKind INSTANCE = new AdaptableKind();

    protected AdaptableKind() {
    }
  }
}