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


package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable;

import java.util.LinkedHashSet;
import java.util.Optional;
import java.util.Set;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;

@Deprecated
public class EmbeddedMontiArcMathModelNameCalculator extends de.monticore.CommonModelNameCalculator {

  @Override
  public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
    final Set<String> calculatedModelNames = new LinkedHashSet<>();

//      if (EMAMCompilationUnitSymbol.KIND.isKindOf(kind)) {
//        calculatedModelNames.addAll(calculateModelNamesForEMAMCompilationUnit(name));
//      }

    return calculatedModelNames;
  }

  protected Set<String> calculateModelNamesForEMAMCompilationUnit(String name) {
    final Set<String> modelNames = new LinkedHashSet<>();
    modelNames.add(name);
    return modelNames;
  }


}
