package de.monticore.lang.monticar.semantics.solver.solutionValidation;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.Constants;
import de.monticore.lang.monticar.semantics.ExecutionSemantics;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import static org.junit.Assert.*;

public class SolutionValidationTest {

    @Test
    public void testValid() {
        Log.init();
        String model = "de.monticore.lang.monticar.semantics.loops.solutionValidation";

        TaggingResolver symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                Constants.SYNTHESIZED_COMPONENTS_ROOT);
        EMAComponentInstanceSymbol component =
                symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        ExecutionSemantics executionSemantics = new ExecutionSemantics(symTab, component);
        executionSemantics.setResolveLoops(true);
        executionSemantics.setHandleArtificialLoops(true);
        executionSemantics.setSolveSymbolicLoops(true);
        executionSemantics.setSolveSymbolicSpecification(true);

        executionSemantics.setWarnLoops(false);
        executionSemantics.setWarnArtificialLoops(false);
        executionSemantics.setLogSymbolicSolve(false);

        executionSemantics.addExecutionSemantics();
    }

}