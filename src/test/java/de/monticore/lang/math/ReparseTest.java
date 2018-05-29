/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math._symboltable.MathLanguage;

import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Paths;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * Created by Dennis on 08.03.2017.
 */
public class ReparseTest {
    private String FILEENDING = ".m";
    private String GENERATEDTARGETPATH = "src/test/resources"; //"target/generated-test-sources"; should be used for generated test sources

    //Find out what the math group had in mind when creating this test and change it to fulfill that
    //or remove it if it is not important
    /*@Ignore
    @Test
    public void testAll() throws Exception {


        String[] testFiles = {"Assignments", "ForLoop", "If3", "Units"};

        for (String file : testFiles) {
            generate(file);
            Executable a = Generator.getInstanceOfGeneratedClass(file);
            a.execute();
        }

        Scope input = createSymTab("src/test/resources");
        Scope output = createSymTab(GENERATEDTARGETPATH);

        checkAssignments(input, output);
        checkForLoop(input, output);
        checkIf3(input, output);
        checkUnits(input, output);

    }

    private void generate(String fileName) {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        try {
            g.generate(model, fileName);
        } catch (IOException e) {
            // System.out.println("Error: file " + fileName + " does not exist");
        }
    }

    private void checkAssignments(Scope input, Scope output) {
        //Q A
        final MathVariableDeclarationSymbol mathSymbol1 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Assignments.A", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol2 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.AssignmentsOutput.A", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        assertNotNull(mathSymbol2);
        assertEquals(((JSValue) mathSymbol1.getValue()).getRealNumber(), ((JSValue) mathSymbol2.getValue()).getRealNumber());

        //Q B
        final MathVariableDeclarationSymbol mathSymbol3 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Assignments.B", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol4 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.AssignmentsOutput.B", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol3);
        assertNotNull(mathSymbol4);
        assertEquals(((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(0, 0)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(0, 0)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(0, 1)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(0, 1)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(1, 0)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(1, 0)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(1, 1)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol3.getValue()).getElement(1, 1)).getRealNumber());

        //Q C
        final MathVariableDeclarationSymbol mathSymbol5 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Assignments.C", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol6 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.AssignmentsOutput.C", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol5);
        assertNotNull(mathSymbol6);
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(0, 0)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(0, 0)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(0, 1)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(0, 1)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(1, 0)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(1, 0)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(1, 1)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(1, 1)).getRealNumber());

        //Q D
        final MathVariableDeclarationSymbol mathSymbol7 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Assignments.D", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol8 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.AssignmentsOutput.D", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol7);
        assertNotNull(mathSymbol8);
        assertEquals(((JSValue) mathSymbol7.getValue()).getRealNumber(), ((JSValue) mathSymbol8.getValue()).getRealNumber());
    }

    private void checkForLoop(Scope input, Scope output) {
        //Q sum
        final MathVariableDeclarationSymbol mathSymbol1 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.ForLoop.sum", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol2 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.ForLoopOutput.sum", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        assertNotNull(mathSymbol2);
        assertEquals(((JSValue) mathSymbol1.getValue()).getRealNumber(), ((JSValue) mathSymbol2.getValue()).getRealNumber());
    }

    private void checkIf3(Scope input, Scope output) {
        //Q cond1
        final MathVariableDeclarationSymbol mathSymbol1 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3.cond1", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol2 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3Output.cond1", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        assertNotNull(mathSymbol2);
        assertEquals(((JSValue) mathSymbol1.getValue()).getRealNumber(), ((JSValue) mathSymbol2.getValue()).getRealNumber());

        //Q cond2
        final MathVariableDeclarationSymbol mathSymbol3 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3.cond2", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol4 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3Output.cond2", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol3);
        assertNotNull(mathSymbol4);
        assertEquals(((JSValue) mathSymbol3.getValue()).getRealNumber(), ((JSValue) mathSymbol4.getValue()).getRealNumber());

        //Q result
        final MathVariableDeclarationSymbol mathSymbol5 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3.result", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol6 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3Output.result", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol5);
        assertNotNull(mathSymbol6);
        assertEquals(((JSValue) mathSymbol5.getValue()).getRealNumber(), ((JSValue) mathSymbol6.getValue()).getRealNumber());

        //B bool
        final MathVariableDeclarationSymbol mathSymbol7 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3.bool", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol8 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.If3Output.bool", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol7);
        assertNotNull(mathSymbol8);
        assertEquals(((JSValue) mathSymbol7.getValue()).getRealNumber(), ((JSValue) mathSymbol8.getValue()).getRealNumber());
    }

    private void checkUnits(Scope input, Scope output) {
        //Q A
        final MathVariableDeclarationSymbol mathSymbol1 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.A", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol2 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.A", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        assertNotNull(mathSymbol2);
        assertEquals(((JSValue) mathSymbol1.getValue()).getRealNumber(), ((JSValue) mathSymbol2.getValue()).getRealNumber());
        assertEquals((mathSymbol1.getValue()).getUnit(), (mathSymbol2.getValue()).getUnit());

        //Q B
        final MathVariableDeclarationSymbol mathSymbol3 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.B", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol4 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.B", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol3);
        assertNotNull(mathSymbol4);
        assertEquals(((JSValue) mathSymbol3.getValue()).getRealNumber(), ((JSValue) mathSymbol4.getValue()).getRealNumber());
        assertEquals((mathSymbol3.getValue()).getUnit(), (mathSymbol4.getValue()).getUnit());

        //Q C
        final MathVariableDeclarationSymbol mathSymbol5 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.C", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol6 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.C", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol5);
        assertNotNull(mathSymbol6);
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(0, 0)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(0, 0)).getRealNumber());
        assertEquals(((JSValue) ((JSMatrix) mathSymbol5.getValue()).getElement(0, 1)).getRealNumber(), ((JSValue) ((JSMatrix) mathSymbol6.getValue()).getElement(0, 1)).getRealNumber());
        assertEquals(((JSMatrix) mathSymbol5.getValue()).getElement(0, 0).getUnit(), ((JSMatrix) mathSymbol6.getValue()).getElement(0, 0).getUnit());
        assertEquals(((JSMatrix) mathSymbol5.getValue()).getElement(0, 1).getUnit(), ((JSMatrix) mathSymbol6.getValue()).getElement(0, 1).getUnit());

        //Q F
        final MathVariableDeclarationSymbol mathSymbol7 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.F", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbol8 = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.F", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol7);
        assertNotNull(mathSymbol8);
        assertEquals(((JSValue) mathSymbol7.getValue()).getRealNumber(), ((JSValue) mathSymbol8.getValue()).getRealNumber());
        assertEquals((mathSymbol7.getValue()).getUnit(), (mathSymbol8.getValue()).getUnit());

        //Q D
        final MathVariableDeclarationSymbol mathSymbol9 = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.D", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbolA = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.D", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol9);
        assertNotNull(mathSymbolA);
        assertEquals(((JSValue) mathSymbol9.getValue()).getRealNumber(), ((JSValue) mathSymbolA.getValue()).getRealNumber());
        assertEquals((mathSymbol9.getValue()).getUnit(), (mathSymbolA.getValue()).getUnit());

        //Q E
        final MathVariableDeclarationSymbol mathSymbolB = input.<MathVariableDeclarationSymbol>resolve("expectedOutput.Units.E", MathVariableDeclarationSymbol.KIND).orElse(null);
        final MathVariableDeclarationSymbol mathSymbolC = output.<MathVariableDeclarationSymbol>resolve("expectedOutput.UnitsOutput.E", MathVariableDeclarationSymbol.KIND).orElse(null);
        assertNotNull(mathSymbolB);
        assertNotNull(mathSymbolC);
        assertEquals(((JSValue) mathSymbolB.getValue()).getRealNumber(), ((JSValue) mathSymbolC.getValue()).getRealNumber());
        assertEquals((mathSymbolB.getValue()).getUnit(), (mathSymbolC.getValue()).getUnit());
    }
*/
    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new MathLanguage());
        final ModelPath mp = new ModelPath();

        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }


}
