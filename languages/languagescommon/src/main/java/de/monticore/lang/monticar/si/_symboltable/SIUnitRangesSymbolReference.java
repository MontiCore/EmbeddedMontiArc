/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRanges;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class SIUnitRangesSymbolReference extends SIUnitRangesSymbol implements MCTypeReference<SIUnitRangesSymbol> {

    protected int dimension = 0;

    public SIUnitRangesSymbolReference(final String name) {
        super(name);
    }

    public SIUnitRangesSymbolReference(final String name, List<ASTRange> ranges) {
        super(name, ranges);
    }

    public static SIUnitRangesSymbolReference constructSIUnitRangesSymbolReference(ASTRange astType) {
        String name = "SIUnitRangesType";
        List<ASTRange> ranges = new ArrayList<ASTRange>();
        ranges.add(astType);
        return new SIUnitRangesSymbolReference(name, ranges);
    }

    public static SIUnitRangesSymbolReference constructSIUnitRangesSymbolReference(ASTRanges astType) {
        return constructSIUnitRangesSymbolReference(astType.getRangeList());
    }

    public static SIUnitRangesSymbolReference constructSIUnitRangesSymbolReference(List<ASTRange> astRanges) {
        return new SIUnitRangesSymbolReference("SIUnitRangesType", astRanges);
    }

    @Override
    public SIUnitRangesSymbol getReferencedSymbol() {
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
