/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import de.monticore.lang.math._symboltable.JSValue;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.resolving.SymbolAdapter;

import java.util.Arrays;
import java.util.Collections;

import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 */
public class ResolutionDeclarationSymbol2MathVariableDeclarationSymbol extends MathVariableDeclarationSymbol
        implements SymbolAdapter<ResolutionDeclarationSymbol> {

    private final ResolutionDeclarationSymbol adaptee;

    public ResolutionDeclarationSymbol2MathVariableDeclarationSymbol(ResolutionDeclarationSymbol ps, ASTUnitNumberResolution unitNumberResolution) {
        super(ps.getName(),
                new JSValue(doubleToRational(unitNumberResolution.getNumber().get())));

        this.adaptee = ps;
    }

    @Override
    public ResolutionDeclarationSymbol getAdaptee() {
        return adaptee;
    }

}
