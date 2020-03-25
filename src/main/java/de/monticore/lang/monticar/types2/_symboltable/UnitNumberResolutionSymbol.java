/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.types2._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 */
public class UnitNumberResolutionSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final UnitNumberResolutionKind KIND = UnitNumberResolutionKind.INSTANCE;

    ASTUnitNumberResolution unitNumberResolution;

    protected UnitNumberResolutionSymbol(String name, ASTUnitNumberResolution astUnitNumberResolution) {
        super(name, KIND);
        this.unitNumberResolution = astUnitNumberResolution;
    }

    public ASTUnitNumberResolution getUnitNumberResolution() {
        return unitNumberResolution;
    }

    public void setUnitNumberResolution(ASTUnitNumberResolution unitNumberResolution) {
        this.unitNumberResolution = unitNumberResolution;
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
