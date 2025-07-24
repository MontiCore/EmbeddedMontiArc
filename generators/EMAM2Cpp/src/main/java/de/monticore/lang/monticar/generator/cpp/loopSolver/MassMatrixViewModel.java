/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.EquationSystemFunction;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;

import java.util.ArrayList;
import java.util.List;

public class MassMatrixViewModel extends ViewModelBase {

    private final String name;
    private List<String> massMatrixDiag = new ArrayList<>();

    public MassMatrixViewModel(String name, SemiExplicitForm semiExplicitForm) {
        this.name = name;
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getY()) massMatrixDiag.add("1");
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getZ()) massMatrixDiag.add("0");
    }

    public List<String> getMassMatrixDiag() {
        return massMatrixDiag;
    }

    public String getName() {
        return name;
    }
}
