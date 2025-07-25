/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 */
public class SIUnitSymbol extends CommonSymbol implements MCTypeSymbol {
    public static final SIUnitKind KIND = SIUnitKind.INSTANCE;

    Unit unit;
    Rational number;

    protected SIUnitSymbol(String name, ASTNumberWithUnit ASTNumberWithUnit) {
        super(name, KIND);
        this.unit = ASTNumberWithUnit.getUnit();
        this.number = doubleToRational(ASTNumberWithUnit.getNumber().get());
    }

    public Unit getUnit() {
        return this.unit;
    }

    public void setUnit(Unit unit) {
        this.unit = unit;
    }

    public Rational getNumber() {
        return this.number;
    }

    public void setNumber(Rational number) {
        this.number = number;
    }

    @Override
    public List<? extends MCTypeSymbol> getFormalTypeParameters() {
        return null;
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
