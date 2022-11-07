/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.streamunits._symboltable.*;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.lang.monticar.streamunits._symboltable.StreamInstruction;
import de.monticore.lang.monticar.streamunits._symboltable.StreamValues;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.nio.file.Paths;
import java.util.Collection;

import static de.monticore.lang.monticar.ParserStreamUnitsTest.ENABLE_FAIL_QUICK;
import static org.junit.Assert.*;

/**
 * Created by Sascha on 16.05.2017.
 */
public class SymtabStreamUnitsTest {
    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new StreamUnitsLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.init();
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testResolveComponentStreamUnitsSymbol() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");
        Log.debug(symTab.toString(), "SymTab:");
        ComponentStreamUnitsSymbol comp = symTab.<ComponentStreamUnitsSymbol>resolve(
                "basicLibrary.AndTest", ComponentStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(comp.getNamedStream("in1"));
    }

    @Test
    public void testResolveNamedStreamUnitsSymbol() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "advancedLibrary.RSFlipFlopTest.In_S", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
    }


    @Test
    public void testResolvePrecisionStreamUnit() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "advancedLibrary.CounterTest.in1", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(0);

        ASTNumberWithUnit percision = (ASTNumberWithUnit) instruction.getStreamValue().get().getPrecision();
        assertEquals(0.5, percision.getNumber().get(), 0);
        assertFalse(instruction.getStreamCompare().isPresent());
    }

    @Test
    public void testForwardStreamUnit() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "test.TestForward.direction", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        assertEquals(5, namedStreamSymbol.getValueSize());
        for (int i = 0; i < 5; ++i) {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(0);
            assertEquals("FORWARD", (instruction.getStreamValue().get()).getValue().toString());
            assertFalse(instruction.getStreamCompare().isPresent());
            assertFalse(instruction.getStreamValueAtTick().isPresent());
        }


        namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "test.TestForward.yPosition", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        assertEquals(5, namedStreamSymbol.getValueSize());
        {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(0);
            assertEquals("-", (instruction.getStreamValue().get()).getValue().toString());
            assertFalse(instruction.getStreamCompare().isPresent());
            assertFalse(instruction.getStreamValueAtTick().isPresent());
        }

        {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(1);
            assertEquals("yPosition(2)", (instruction.getStreamCompare().get()).getLeft().toString());
            assertEquals("yPosition(1)", (instruction.getStreamCompare().get()).getRight().toString());
            assertEquals(">", (instruction.getStreamCompare().get()).getOperator().toString());
        }

        {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(2);
            assertEquals("yPosition(3)", (instruction.getStreamCompare().get()).getLeft().toString());
            assertEquals("yPosition(2)", (instruction.getStreamCompare().get()).getRight().toString());
            assertEquals(">", (instruction.getStreamCompare().get()).getOperator().toString());
        }
        {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(3);
            assertEquals("yPosition(4)", (instruction.getStreamCompare().get()).getLeft().toString());
            assertEquals("yPosition(3)", (instruction.getStreamCompare().get()).getRight().toString());
            assertEquals(">", (instruction.getStreamCompare().get()).getOperator().toString());
        }
        {
            StreamInstruction instruction = (StreamInstruction) namedStreamSymbol.getValue(4);
            assertEquals("yPosition(5)", (instruction.getStreamCompare().get()).getLeft().toString());
            assertEquals("yPosition(4)", (instruction.getStreamCompare().get()).getRight().toString());
            assertEquals(">", (instruction.getStreamCompare().get()).getOperator().toString());
        }


    }

    @Test
    public void testResolveMatrixStream() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "emamtest.TestMatrixStream.direction", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        StreamInstruction streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(0);
        assertTrue(streamInstruction.getStreamValues().isPresent());
        StreamValues streamValues = streamInstruction.getStreamValues().get();
        assertEquals(1, streamValues.getRowDimension());
        assertEquals(3, streamValues.getColumnDimension());
        assertEquals("1.0", streamValues.getStreamValue(0, 0).toString());
        assertEquals("0.0", streamValues.getStreamValue(0, 1).toString());
        assertEquals("0.0", streamValues.getStreamValue(0, 2).toString());


        streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(1);
        assertTrue(streamInstruction.getStreamValues().isPresent());
        streamValues = streamInstruction.getStreamValues().get();
        assertEquals(1, streamValues.getRowDimension());
        assertEquals(3, streamValues.getColumnDimension());
        assertEquals("0.0", streamValues.getStreamValue(0, 0).toString());
        assertEquals("1.0", streamValues.getStreamValue(0, 1).toString());
        assertEquals("0.0", streamValues.getStreamValue(0, 2).toString());


        streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(2);
        assertTrue(streamInstruction.getStreamValues().isPresent());
        streamValues = streamInstruction.getStreamValues().get();
        assertEquals(1, streamValues.getRowDimension());
        assertEquals(3, streamValues.getColumnDimension());
        assertEquals("0.0", streamValues.getStreamValue(0, 0).toString());
        assertEquals("0.0", streamValues.getStreamValue(0, 1).toString());
        assertEquals("1.0", streamValues.getStreamValue(0, 2).toString());

    }

    @Test
    public void testResolveMatrixStreamWithFile() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "emamtest.TestMatrixStreamWithFile.direction", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        StreamInstruction streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(2);
        assertTrue(streamInstruction.getStreamValues().isPresent());
        StreamValues streamValues = streamInstruction.getStreamValues().get();
        assertEquals(1, streamValues.getRowDimension());
        assertEquals(3, streamValues.getColumnDimension());
        assertEquals("1.0", streamValues.getStreamValue(0, 0).toString());
        assertEquals("0.0", streamValues.getStreamValue(0, 1).toString());
        assertEquals("1.0", streamValues.getStreamValue(0, 2).toString());
    }

    @Test
    public void testResolveCubeStream() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "advancedLibrary.ImageTest.in1", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        StreamInstruction streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(0);
        assertTrue(streamInstruction.getStreamValues().isPresent());
        StreamValues streamValues = streamInstruction.getStreamValues().get();
        assertEquals(3, streamValues.getDepthDimension());
        assertEquals(3, streamValues.getRowDimension());
        assertEquals(3, streamValues.getColumnDimension());
        assertEquals(0.1, streamValues.getElementTolerance(), 0);
        assertEquals("1.0", streamValues.getStreamValue(0, 0,0).toString());
        assertEquals("3.0", streamValues.getStreamValue(0, 1,1).toString());
        assertEquals("4.0", streamValues.getStreamValue(0, 2,1).toString());
    }

    @Test
    public void testResolveImagePath() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams");

        NamedStreamUnitsSymbol namedStreamSymbol = symTab.<NamedStreamUnitsSymbol>resolve(
                "basicLibrary.AddTest.in2", NamedStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
        StreamInstruction streamInstruction = (StreamInstruction) namedStreamSymbol.getValue(2);
        assertTrue(streamInstruction.getImagePath().isPresent());
        ImagePath imagePath = streamInstruction.getImagePath().get();
        assertEquals(imagePath.getImagePath(), "/src/test/resources/images/0.png");
    }


    @Ignore("ModelPath#resolveModel does not support loading a collection, which should be done when resolving many")
    @Test
    public void testResolveNamedManyStreamUnitsSymbol() {
        Scope symTab = createSymTab("src/test/resources/unitstreams/streams", "src/test/resources/unitstreams/streams2");

        Collection<NamedStreamUnitsSymbol> namedStreamSymbols = symTab.<NamedStreamUnitsSymbol>resolveMany(
                "advancedLibrary.RSFlipFlopTest.In_S", NamedStreamUnitsSymbol.KIND);
        assertEquals(2, namedStreamSymbols.size());
    }
}
