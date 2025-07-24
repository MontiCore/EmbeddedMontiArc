/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */


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
