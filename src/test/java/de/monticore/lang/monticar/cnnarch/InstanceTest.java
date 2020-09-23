/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static de.monticore.lang.monticar.cnnarch.ParserTest.ENABLE_FAIL_QUICK;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class InstanceTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testInstanceCreation(){
        Scope symTab = createSymTab("src/test/resources/architectures");
        CNNArchCompilationUnitSymbol compilationUnitSymbol = symTab.<CNNArchCompilationUnitSymbol>resolve(
                "SequentialAlexnet",
                CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull(compilationUnitSymbol);

        compilationUnitSymbol.setParameter("classes", 100);
        ArchitectureSymbol instance1 = compilationUnitSymbol.resolve();


        CNNArchCompilationUnitSymbol compilationUnit2 = compilationUnitSymbol.preResolveDeepCopy();
        compilationUnit2.setParameter("img_height", 200);
        compilationUnit2.setParameter("img_width", 210);
        ArchitectureSymbol instance2 = compilationUnit2.resolve();

        int width1 = ((IODeclarationSymbol) instance1.getInputs().get(0).getDeclaration()).getType().getWidth();
        int height1 = ((IODeclarationSymbol) instance1.getInputs().get(0).getDeclaration()).getType().getHeight();
        int channels1 = ((IODeclarationSymbol) instance1.getOutputs().get(0).getDeclaration()).getType().getChannels();
        int lastLayerChannels1 = instance1.getOutputs().get(0).getInputTypes().get(0).getChannels();

        int width2 = ((IODeclarationSymbol) instance2.getInputs().get(0).getDeclaration()).getType().getWidth();
        int height2 = ((IODeclarationSymbol) instance2.getInputs().get(0).getDeclaration()).getType().getHeight();
        int channels2 = ((IODeclarationSymbol) instance2.getOutputs().get(0).getDeclaration()).getType().getChannels();
        int lastLayerChannels2 = instance2.getOutputs().get(0).getInputTypes().get(0).getChannels();

        assertEquals(224, width1);
        assertEquals(224, height1);
        assertEquals(100, channels1);
        assertEquals(100, lastLayerChannels1);
        assertEquals(210, width2);
        assertEquals(200, height2);
        assertEquals(10, channels2);
        assertEquals(10, lastLayerChannels2);
    }

    @Test
    public void testInstanceCreation2(){
        Scope symTab = createSymTab("src/test/resources/valid_tests");
        CNNArchCompilationUnitSymbol compilationUnitSymbol = symTab.<CNNArchCompilationUnitSymbol>resolve(
                "ResNeXt50_InstanceTest",
                CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull(compilationUnitSymbol);


        compilationUnitSymbol.setParameter("cardinality", 32);
        ArchitectureSymbol instance1 = compilationUnitSymbol.resolve();


        CNNArchCompilationUnitSymbol compilationUnit2 = compilationUnitSymbol.preResolveDeepCopy();
        compilationUnit2.setParameter("cardinality", 2);
        ArchitectureSymbol instance2 = compilationUnit2.resolve();
        ArchRangeExpressionSymbol range1 = (ArchRangeExpressionSymbol) ((LayerSymbol)(((CompositeElementSymbol)((CompositeElementSymbol)((CompositeElementSymbol)((CompositeElementSymbol) instance1.getStreams().get(0)).getElements().get(5).getResolvedThis().get()).getElements().get(0)).getElements().get(0)).getElements().get(0)))
                .getArgument(AllPredefinedVariables.PARALLEL_ARG_NAME).get().getRhs();
        ArchRangeExpressionSymbol range2 = (ArchRangeExpressionSymbol) ((LayerSymbol)(((CompositeElementSymbol)((CompositeElementSymbol)((CompositeElementSymbol)((CompositeElementSymbol) instance2.getStreams().get(0)).getElements().get(5).getResolvedThis().get()).getElements().get(0)).getElements().get(0)).getElements().get(0)))
                .getArgument(AllPredefinedVariables.PARALLEL_ARG_NAME).get().getRhs();

        assertEquals(32, range1.getElements().get().size());
        assertEquals(2, range2.getElements().get().size());

    }
}
