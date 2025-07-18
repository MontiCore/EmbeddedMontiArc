/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.semantics.setup.Delegate;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

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

    public SemiExplicitForm indexed() {
        Map<String, String> nameMapping = new HashMap<>();

        SemiExplicitFormBuilder builder = SemiExplicitFormBuilder.aSemiExplicitForm();

        int index = 0;
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : getY())
            nameMapping.put(emamSymbolicVariableSymbol.getName(), String.format("x[%s]", index++));
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : getZ())
            nameMapping.put(emamSymbolicVariableSymbol.getName(), String.format("x[%s]", index++));

        builder.setY(getY());
        builder.setZ(getZ());
        builder.setInitialValues(getInitialValues());
        builder.setInitialGuesses(getInitialGuesses());

        for (MathExpressionSymbol mathExpressionSymbol : getF()) {
            MathExpressionSymbol copy = Delegate.copyMathExpressionSymbol(mathExpressionSymbol);
            NameReplacer.replaceNames(copy, n -> nameMapping.get(n));
            builder.addF(copy);
        }

        for (EquationSystemFunction function : getG()) {
            if (function instanceof ExplicitFunction) {
                EMAMEquationSymbol copy = Delegate.copyMathExpressionSymbol(((ExplicitFunction) function).getEquation());
                NameReplacer.replaceNames(copy, n -> nameMapping.get(n));
                builder.addG(new ExplicitFunction(copy));
            } else
                builder.addG(function);
        }

        return builder.build();
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
