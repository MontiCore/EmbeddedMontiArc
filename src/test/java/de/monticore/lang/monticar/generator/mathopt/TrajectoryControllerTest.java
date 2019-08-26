/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mathopt;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class TrajectoryControllerTest extends AbstractSymtabTest {

    @Test
    public void generateTrajectoryControllerMPCBicycle() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/mathopt");
        EMAComponentInstanceSymbol componentInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("de.rwth.monticar.mpc.trajectoryControllerMPC", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        GeneratorCPP generator = new GeneratorCPP();
        generator.setGenerateAutopilotAdapter(true);
        generator.setGenerateServerWrapper(false);
        generator.setGenerateCMake(true);
        generator.setGenerationTargetPath("./target/generated-sources-cpp/TrajectoryControllerMPC/src/");

        //TODO: fix errors
//        generator.setPreferedSolver(Solver.Ipopt);
//        generator.getSolverOptions().put("String  derivative_test", "none");
//        generator.getSolverOptions().put("Integer print_level", "3");
//        generator.getSolverOptions().put("Numeric tol", "0.01");
//        generator.getSolverOptions().put("Integer max_iter", "50");
        List<File> files = generator.generateFiles(componentInstanceSymbol, symtab);
    }

}
