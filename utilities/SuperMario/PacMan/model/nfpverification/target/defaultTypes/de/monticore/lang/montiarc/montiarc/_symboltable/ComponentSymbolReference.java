package de.monticore.lang.montiarc.montiarc._symboltable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import de.monticore.lang.montiarc.helper.SymbolPrinter;
import de.monticore.lang.montiarc.tagging._symboltable.TaggingScopeSpanningSymbol;
import de.monticore.symboltable.references.SymbolReference;
import de.monticore.symboltable.types.references.ActualTypeArgument;

/**
 * method names for default types
 */
public class ComponentSymbolReference extends ComponentSymbol implements
        SymbolReference<ComponentSymbol> {

  public ComponentSymbol getReferencedSymbol() ;

}
