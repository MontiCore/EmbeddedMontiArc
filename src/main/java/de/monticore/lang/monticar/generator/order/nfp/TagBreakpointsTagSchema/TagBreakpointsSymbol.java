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
package de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Arrays;
import java.util.stream.Collectors;

/**
 * Created by ernst on 13.07.2016.
 */
public class TagBreakpointsSymbol extends TagSymbol {
  public static final TagBreakpointsSymbolKind KIND = TagBreakpointsSymbolKind.INSTANCE;

  public TagBreakpointsSymbol(Double[] breakpoints) {
    super(KIND, breakpoints);
  }

  @Override
  public String toString() {
    return (Arrays.stream(getBreakpoints()).map(b -> b.toString()).collect(Collectors.joining(",")));
  }

  public Double[] getBreakpoints() {
    return getValues().toArray(new Double[getValues().size()]);
  }

  public static class TagBreakpointsSymbolKind extends TagKind {

    public static final TagBreakpointsSymbolKind INSTANCE = new TagBreakpointsSymbolKind();

    protected TagBreakpointsSymbolKind() {}
  }
}


