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
package de.monticore.lang.monticar.struct._symboltable.validation;

import de.monticore.lang.monticar.struct.Utils;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class AllStructReferencesAreRecursivelyResolvableCheckerTest {

    private Scope symTab;
    private AllStructReferencesAreRecursivelyResolvableChecker checker;

    @Before
    public void setup() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        symTab = Utils.createSymTab("src/test/resources");
        checker = new AllStructReferencesAreRecursivelyResolvableChecker();
    }

    @Test
    public void testValidStructs() {
        String[] validStructs = new String[]{
                "test.symtable.MyFancyStruct1",
                "test.symtable.sub1.S1",
                "test.symtable.sub2.T1",
                "test.symtable.sub3.K1"
        };
        for (String structFullName : validStructs) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            Assert.assertTrue(checker.isValid(struct));
        }
    }

    @Test
    public void testSelfReferences() {
        for (int i = 1; i <= 4; i++) {
            String structFullName = String.format("test.symtable.ErrSelfReference%s", i);
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            Assert.assertFalse(checker.isValid(struct));
        }
    }

    @Test
    public void testCycles() {
        String[] structsWithCycles = new String[]{
                "test.symtable.sub1.ErrCycle1",
                "test.symtable.sub2.ErrCycle1",
                "test.symtable.sub3.ErrCycle1",
                "test.symtable.sub1.ErrCycle2",
                "test.symtable.sub2.ErrCycle2",
                "test.symtable.sub3.ErrCycle2"
        };
        for (String structFullName : structsWithCycles) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            Assert.assertFalse(checker.isValid(struct));
        }
    }

    @Test
    public void testNonExistentReferences() {
        String[] structsWithNonExistentReferences = new String[]{
                "test.symtable.ErrNonExistentReferences"
        };
        for (String structFullName : structsWithNonExistentReferences) {
            StructSymbol struct = symTab.<StructSymbol>resolve(structFullName, StructSymbol.KIND).orElse(null);
            Assert.assertNotNull(struct);
            Assert.assertFalse(checker.isValid(struct));
        }
    }
}
