/* (c) https://github.com/MontiCore/monticore */


package de.monticore.lang.embeddedmontiarc.application._symboltable;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

/**
 * Represents a reference of {@link EMAAplCompilationUnitSymbol}.
 */
public class EMAAplCompilationUnitSymbolReference extends EMAAplCompilationUnitSymbol implements SymbolReference<EMAAplCompilationUnitSymbol> {
  protected final SymbolReference<EMAAplCompilationUnitSymbol> reference;

  public EMAAplCompilationUnitSymbolReference(final String name, final Scope enclosingScopeOfReference) {
    super(name);
    reference = new CommonSymbolReference<>(name, EMAAplCompilationUnitSymbol.KIND, enclosingScopeOfReference);
  }

  /*
   * Methods of SymbolReference interface
   */

  @Override
  public EMAAplCompilationUnitSymbol getReferencedSymbol() {
    return reference.getReferencedSymbol();
  }

  @Override
  public boolean existsReferencedSymbol() {
    return reference.existsReferencedSymbol();
  }

  @Override
  public boolean isReferencedSymbolLoaded() {
    return reference.isReferencedSymbolLoaded();
  }

  /*
  * Methods of Symbol interface
  */

  @Override
  public String getName() {
    return getReferencedSymbol().getName();
  }

  @Override
  public String getFullName() {
    return getReferencedSymbol().getFullName();
  }

  @Override
  public void setEnclosingScope(MutableScope scope) {
    getReferencedSymbol().setEnclosingScope(scope);
  }

  @Override
  public Scope getEnclosingScope() {
    return getReferencedSymbol().getEnclosingScope();
  }

  @Override
  public AccessModifier getAccessModifier() {
    return getReferencedSymbol().getAccessModifier();
  }

  @Override
  public void setAccessModifier(AccessModifier accessModifier) {
    getReferencedSymbol().setAccessModifier(accessModifier);
  }


}

