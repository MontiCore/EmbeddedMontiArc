/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

/**
 */
public class MathVariableDeclarationSymbolReference extends MathVariableDeclarationSymbol implements SymbolReference<MathVariableDeclarationSymbol> {
    protected final SymbolReference<MathVariableDeclarationSymbol> reference;

    public MathVariableDeclarationSymbolReference(final String name, final Scope enclosingScopeOfReference) {
        super(name, null, null, null);
        reference = new CommonSymbolReference<>(name, KIND, enclosingScopeOfReference);
    }

  /*
   * Methods of SymbolReference interface
   */

    @Override
    public MathVariableDeclarationSymbol getReferencedSymbol() {

        try {
            return reference.getReferencedSymbol();
        } catch (Exception ex) {
            return null;

        }
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
        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getName();
        return super.getName();
    }

    @Override
    public String getFullName() {
        if (getReferencedSymbol() != null) {
            return getReferencedSymbol().getFullName();
        }
        return super.getFullName();
    }

    @Override
    public void setEnclosingScope(MutableScope scope) {
        if (getReferencedSymbol() != null)
            getReferencedSymbol().setEnclosingScope(scope);
        super.setEnclosingScope(scope);
    }

    @Override
    public Scope getEnclosingScope() {

        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getEnclosingScope();
        return super.getEnclosingScope();
    }

    @Override
    public AccessModifier getAccessModifier() {

        if (getReferencedSymbol() != null)
            return getReferencedSymbol().getAccessModifier();
        return super.getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        if (getReferencedSymbol() != null)
            getReferencedSymbol().setAccessModifier(accessModifier);
        super.setAccessModifier(accessModifier);
    }
}
