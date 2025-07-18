/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;

public final class SemiExplicitFormBuilder {
    private ArrayList<EMAMSymbolicVariableSymbol> y = new ArrayList();
    private ArrayList<EMAMSymbolicVariableSymbol> z = new ArrayList();
    private ArrayList<MathExpressionSymbol> f = new ArrayList();
    private ArrayList<EquationSystemFunction> g = new ArrayList();
    private ArrayList<MathExpressionSymbol> initialValues = new ArrayList();
    private ArrayList<MathExpressionSymbol> initialGuesses = new ArrayList();

    private SemiExplicitFormBuilder() {
    }

    public static SemiExplicitFormBuilder aSemiExplicitForm() {
        return new SemiExplicitFormBuilder();
    }

    public SemiExplicitFormBuilder setY(ArrayList<EMAMSymbolicVariableSymbol> y) {
        this.y = y;
        return this;
    }

    public SemiExplicitFormBuilder setZ(ArrayList<EMAMSymbolicVariableSymbol> z) {
        this.z = z;
        return this;
    }

    public SemiExplicitFormBuilder setF(ArrayList<MathExpressionSymbol> f) {
        this.f = f;
        return this;
    }

    public SemiExplicitFormBuilder setG(ArrayList<EquationSystemFunction> g) {
        this.g = g;
        return this;
    }

    public SemiExplicitFormBuilder setInitialValues(ArrayList<MathExpressionSymbol> initialValues) {
        this.initialValues = initialValues;
        return this;
    }

    public SemiExplicitFormBuilder setInitialGuesses(ArrayList<MathExpressionSymbol> initialGuesses) {
        this.initialGuesses = initialGuesses;
        return this;
    }

    public SemiExplicitFormBuilder addY(EMAMSymbolicVariableSymbol y) {
        this.y.add(y);
        return this;
    }

    public SemiExplicitFormBuilder addZ(EMAMSymbolicVariableSymbol z) {
        this.z.add(z);
        return this;
    }

    public SemiExplicitFormBuilder addF(MathExpressionSymbol f) {
        this.f.add(f);
        return this;
    }

    public SemiExplicitFormBuilder addG(EquationSystemFunction g) {
        this.g.add(g);
        return this;
    }

    public SemiExplicitFormBuilder addInitialValue(MathExpressionSymbol initialValue) {
        this.initialValues.add(initialValue);
        return this;
    }

    public SemiExplicitFormBuilder addInitialGuess(MathExpressionSymbol initialGuess) {
        this.initialGuesses.add(initialGuess);
        return this;
    }

    public ArrayList<EMAMSymbolicVariableSymbol> getY() {
        return y;
    }

    public ArrayList<EMAMSymbolicVariableSymbol> getZ() {
        return z;
    }

    public ArrayList<MathExpressionSymbol> getF() {
        return f;
    }

    public ArrayList<EquationSystemFunction> getG() {
        return g;
    }

    public SemiExplicitForm build() {
        return new SemiExplicitForm(y, z, f, g, initialValues, initialGuesses);
    }
}
