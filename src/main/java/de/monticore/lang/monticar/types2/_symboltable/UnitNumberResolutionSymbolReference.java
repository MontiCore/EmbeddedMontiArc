/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.types2._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.List;

/**
 */
public class UnitNumberResolutionSymbolReference extends UnitNumberResolutionSymbol implements MCTypeReference<UnitNumberResolutionSymbol> {
    protected int dimension = 0;

    public UnitNumberResolutionSymbolReference(final String name, ASTUnitNumberResolution astUnitNumber) {
        super(name, astUnitNumber);
    }

    @Override
    public UnitNumberResolutionSymbol getReferencedSymbol() {
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
