/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 */
public class SIUnitRangesSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final SIUnitRangesSymbolKind KIND = SIUnitRangesSymbolKind.INSTANCE;

    protected List<ASTRange> ranges = new ArrayList<ASTRange>();

    public SIUnitRangesSymbol(String name) {
        super(name, KIND);
    }

    public SIUnitRangesSymbol(String name, List<ASTRange> ranges) {
        super(name, KIND);
        this.ranges.addAll(ranges);
    }

    public ASTRange getRange(int index) {
        if (0 > index || index >= ranges.size())
            Log.error("INDEX IS WRONG");
        return ranges.get(index);
    }

    public int howManyRanges() {
        return ranges.size();
    }

    @Override
    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return Collections.emptyList();
    }

    @Override
    public Optional<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperClass() {
        return Optional.empty();
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getInterfaces() {
        return Collections.emptyList();
    }

    @Override
    public List<? extends MCTypeReference<? extends MCTypeSymbol>> getSuperTypes() {
        return Collections.emptyList();
    }

    @Override
    public boolean isFormalTypeParameter() {
        return false;
    }

    @Override
    public Scope getSpannedScope() {
        return null;
    }
}
