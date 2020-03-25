/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.List;

/**
 */
public class SIUnitSymbolReference extends SIUnitSymbol implements MCTypeReference<SIUnitSymbol> {
    protected int dimension = 0;

    public SIUnitSymbolReference(final String name, ASTNumberWithUnit ASTNumberWithUnit) {
        super(name, ASTNumberWithUnit);
    }

    @Override
    public SIUnitSymbol getReferencedSymbol() {
        return this;
    }

    @Override
    public boolean existsReferencedSymbol() {
        return true;
    }

    @Override
    public boolean isReferencedSymbolLoaded() {
        return true;
    }

    @Override
    public AccessModifier getAccessModifier() {
        return this.getReferencedSymbol().getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        this.getReferencedSymbol().setAccessModifier(accessModifier);
    }

    @Override
    public int getDimension() {
        return dimension;
    }

    @Override
    public void setDimension(int dimension) {
        this.dimension = dimension;
    }

    @Override
    public List<ActualTypeArgument> getActualTypeArguments() {
        return null;
    }

    @Override
    public void setActualTypeArguments(List<ActualTypeArgument> list) {
    }

}
