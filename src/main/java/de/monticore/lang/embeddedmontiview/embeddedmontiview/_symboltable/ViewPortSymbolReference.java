/* (c) https://github.com/MontiCore/monticore */
/* generated from model null*/
/* generated by template symboltable.SymbolReference*/

package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

/**
 * Represents a reference of {@link ViewPortSymbol}.
 */
public class ViewPortSymbolReference extends ViewPortSymbol
    implements SymbolReference<ViewPortSymbol> {
  protected final SymbolReference<ViewPortSymbol> reference;

  public ViewPortSymbolReference(final String name, final Scope definingScopeOfReference) {
    super(name);
    reference = new CommonSymbolReference<>(name, ViewPortSymbol.KIND, definingScopeOfReference);
  }

  /*
   * Methods of SymbolReference interface
   */

  @Override
  public ViewPortSymbol getReferencedSymbol() {
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
  public Scope getEnclosingScope() {
    return getReferencedSymbol().getEnclosingScope();
  }

  @Override
  public void setEnclosingScope(MutableScope scope) {
    getReferencedSymbol().setEnclosingScope(scope);
  }

  @Override
  public AccessModifier getAccessModifier() {
    return getReferencedSymbol().getAccessModifier();
  }

  @Override
  public void setAccessModifier(AccessModifier accessModifier) {
    getReferencedSymbol().setAccessModifier(accessModifier);
  }


  /*
  * Methods of ArcPortSymbol class
  */

  @Override
  public void setDirection(boolean direction) {
    getReferencedSymbol().setDirection(direction);
  }

  @Override
  public boolean isIncoming() {
    return getReferencedSymbol().isIncoming();
  }

  @Override
  public boolean isOutgoing() {
    return getReferencedSymbol().isOutgoing();
  }

}

