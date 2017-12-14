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
package de.monticore.lang.monticar.struct;

import de.monticore.lang.monticar.enumlang._symboltable.EnumConstantDeclarationSymbol;
import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.references.FailedLoadingSymbol;
import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class SymtabTest {

    private Scope symTab;

    @BeforeClass
    public static void setupSpec() {
        Log.getFindings().clear();
    }

    @Before
    public void setup() {
        symTab = Utils.createSymTab("src/test/resources");
    }

    @Test
    public void testFieldNamesAndTypes() {
        StructSymbol struct = symTab.<StructSymbol>resolve("test.symtable.MyFancyStruct1", StructSymbol.KIND).orElse(null);
        Assert.assertNotNull(struct);

        List<StructFieldDefinitionSymbol> fields = new ArrayList<>(struct.getStructFieldDefinitions());
        for (int i = 0, numberOfFields = fields.size(); i < numberOfFields; i++) {
            String expectedFieldName = String.format("f%s", i + 1);
            Assert.assertEquals(expectedFieldName, fields.get(i).getName());
        }

        String[] expectedTypes = new String[]{
                "B",
                "Z",
                "test.symtable.sub1.S1",
                "test.symtable.sub2.T3",
                "test.symtable.sub3.K1",
                "Q",
                "Z",
                "Z",
                "Z",
                "B",
                "C",
                "test.symtable.sub1.S2",
                "test.symtable.sub2.T1",
                "test.symtable.sub3.K2",
                "test.symtable.sub2.T5",
                "Z",
                "test.symtable.sub1.S1"
        };
        for (int i = 0, numberOfFields = fields.size(); i < numberOfFields; i++) {
            StructFieldDefinitionSymbol f = fields.get(i);
            String expectedType = expectedTypes[i];
            Assert.assertEquals(expectedType, f.getType().getReferencedSymbol().getFullName());
        }

        Assert.assertEquals(1, fields.get(11).getType().getDimension());
        Assert.assertEquals(1, fields.get(12).getType().getDimension());
        Assert.assertEquals(1, fields.get(13).getType().getDimension());
        Assert.assertEquals(1, fields.get(14).getType().getDimension());
        Assert.assertEquals(10, fields.get(16).getType().getDimension());
    }

    @Test
    public void testResolveField1() {
        StructFieldDefinitionSymbol f1 = symTab.<StructFieldDefinitionSymbol>resolve("test.symtable.MyFancyStruct1.f1", StructFieldDefinitionSymbol.KIND).orElse(null);
        Assert.assertNotNull(f1);
        Assert.assertEquals("f1", f1.getName());
        Assert.assertEquals("B", f1.getType().getReferencedSymbol().getFullName());
    }

    @Test
    public void testNonExistentReferences() {
        StructSymbol struct = symTab.<StructSymbol>resolve("test.symtable.ErrNonExistentReferences", StructSymbol.KIND).orElse(null);
        Assert.assertNotNull(struct);
        List<StructFieldDefinitionSymbol> fields = new ArrayList<>(struct.getStructFieldDefinitions());
        for (StructFieldDefinitionSymbol f : fields) {
            try {
                f.getType().getReferencedSymbol();
            } catch (FailedLoadingSymbol e) {
                continue;
            }
            Assert.fail();
        }
    }

    @Test
    public void testAmbiguousReferences() {
        String[] fieldsHavingAmbigousReference = new String[]{
                "test.symtable.ErrAmbiguousReference1.err",
                "test.symtable.ErrAmbiguousReference2.err"
        };
        for (String fieldFullName : fieldsHavingAmbigousReference) {
            StructFieldDefinitionSymbol field = symTab.<StructFieldDefinitionSymbol>resolve(fieldFullName, StructFieldDefinitionSymbol.KIND).orElse(null);
            Assert.assertNotNull(field);
            try {
                field.getType().getReferencedSymbol();
            } catch (ResolvedSeveralEntriesException e) {
                continue;
            }
            Assert.fail();
        }
    }

    @Test
    public void testMyStructWithEnum() {
        StructSymbol struct = symTab.<StructSymbol>resolve("test.symtable.MyStructWithEnum", StructSymbol.KIND).orElse(null);
        Assert.assertNotNull(struct);
        ArrayList<StructFieldDefinitionSymbol> fields = new ArrayList<>(struct.getStructFieldDefinitions());
        Assert.assertEquals(3, fields.size());
        StructFieldDefinitionSymbol f3 = fields.get(2);
        Assert.assertEquals("f3", f3.getName());
        MCTypeSymbol typeSymbol = f3.getType().getReferencedSymbol();
        Assert.assertTrue(typeSymbol instanceof EnumDeclarationSymbol);
        EnumDeclarationSymbol enumDecl = (EnumDeclarationSymbol) typeSymbol;
        ArrayList<EnumConstantDeclarationSymbol> enumConstants = new ArrayList<>(enumDecl.getEnumConstants());
        Assert.assertEquals(5, enumConstants.size());
    }
}
