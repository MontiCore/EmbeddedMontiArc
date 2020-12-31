/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;

// see https://de.mathworks.com/help/matlab/math/solve-differential-algebraic-equations-daes.html
public class SemiExplicitForm {
    private ArrayList<EMAMSymbolicVariableSymbol> y;
    private ArrayList<EMAMSymbolicVariableSymbol> z;

    private ArrayList<MathExpressionSymbol> f;
    private ArrayList<EquationSystemFunction> g;

    private ArrayList<MathExpressionSymbol> initialValues;
    private ArrayList<MathExpressionSymbol> initialGuesses;

    public SemiExplicitForm(ArrayList<EMAMSymbolicVariableSymbol> y,
                            ArrayList<EMAMSymbolicVariableSymbol> z,
                            ArrayList<MathExpressionSymbol> f,
                            ArrayList<EquationSystemFunction> g,
                            ArrayList<MathExpressionSymbol> initialValues,
                            ArrayList<MathExpressionSymbol> initialGuesses) {
        this.y = y;
        this.z = z;
        this.f = f;
        this.g = g;
        this.initialValues = initialValues;
        this.initialGuesses = initialGuesses;
    }

    public ArrayList<EMAMSymbolicVariableSymbol> getY() {
        return y;
    }

    public void setY(ArrayList<EMAMSymbolicVariableSymbol> y) {
        this.y = y;
    }

    public ArrayList<EMAMSymbolicVariableSymbol> getZ() {
        return z;
    }

    public void setZ(ArrayList<EMAMSymbolicVariableSymbol> z) {
        this.z = z;
    }

    public ArrayList<MathExpressionSymbol> getF() {
        return f;
    }

    public void setF(ArrayList<MathExpressionSymbol> f) {
        this.f = f;
    }

    public ArrayList<EquationSystemFunction> getG() {
        return g;
    }

    public void setG(ArrayList<EquationSystemFunction> g) {
        this.g = g;
    }

    public ArrayList<MathExpressionSymbol> getInitialValues() {
        return initialValues;
    }

    public void setInitialValues(ArrayList<MathExpressionSymbol> initialValues) {
        this.initialValues = initialValues;
    }

    public ArrayList<MathExpressionSymbol> getInitialGuesses() {
        return initialGuesses;
    }

    public void setInitialGuesses(ArrayList<MathExpressionSymbol> initialGuesses) {
        this.initialGuesses = initialGuesses;
    }
}
