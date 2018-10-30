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
package de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Arrays;
import java.util.stream.Collectors;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagTableSymbol extends TagSymbol {
  public static final TagTableSymbolKind KIND = TagTableSymbolKind.INSTANCE;

  public TagTableSymbol(Double[] table) {
    super(KIND, table);
  }

  @Override
  public String toString() {
    return (Arrays.stream(getTable()).map(t -> t.toString()).collect(Collectors.joining(",")));
  }

  public Double[] getTable() {
    return getValues().toArray(new Double[getValues().size()]);
  }

  public static class TagTableSymbolKind extends TagKind {

    public static final TagTableSymbolKind INSTANCE = new TagTableSymbolKind();

    protected TagTableSymbolKind() {}
  }
}


