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

import de.monticore.lang.monticar.struct._symboltable.StructFieldDefinitionSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.struct.model.type.ScalarStructFieldType;
import de.monticore.lang.monticar.struct.model.type.StructReferenceFieldType;
import de.monticore.lang.monticar.struct.model.type.VectorStructFieldType;
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
        StructFieldDefinitionSymbol f1 = fields.get(0);
        StructFieldDefinitionSymbol f2 = fields.get(1);
        StructFieldDefinitionSymbol f3 = fields.get(2);
        StructFieldDefinitionSymbol f4 = fields.get(3);
        StructFieldDefinitionSymbol f5 = fields.get(4);
        StructFieldDefinitionSymbol f6 = fields.get(5);
        StructFieldDefinitionSymbol f7 = fields.get(6);
        StructFieldDefinitionSymbol f8 = fields.get(7);
        StructFieldDefinitionSymbol f9 = fields.get(8);
        StructFieldDefinitionSymbol f10 = fields.get(9);
        StructFieldDefinitionSymbol f11 = fields.get(10);
        StructFieldDefinitionSymbol f12 = fields.get(11);
        StructFieldDefinitionSymbol f13 = fields.get(12);
        StructFieldDefinitionSymbol f14 = fields.get(13);
        StructFieldDefinitionSymbol f15 = fields.get(14);
        StructFieldDefinitionSymbol f16 = fields.get(15);
        StructFieldDefinitionSymbol f17 = fields.get(16);

        Assert.assertTrue(ScalarStructFieldType.BOOL.equals(f1.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.INTEGRAL.equals(f2.getTypeInfo()));
        Assert.assertEquals(
                "test.symtable.sub1.S1",
                ((StructReferenceFieldType) f3.getTypeInfo()).getReference().getReferencedSymbol().getFullName()
        );
        Assert.assertEquals(
                "test.symtable.sub2.T3",
                ((StructReferenceFieldType) f4.getTypeInfo()).getReference().getReferencedSymbol().getFullName()
        );
        Assert.assertEquals(
                "test.symtable.sub3.K1",
                ((StructReferenceFieldType) f5.getTypeInfo()).getReference().getReferencedSymbol().getFullName()
        );
        Assert.assertTrue(ScalarStructFieldType.RATIONAL.equals(f6.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.INTEGRAL.equals(f7.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.INTEGRAL.equals(f8.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.INTEGRAL.equals(f9.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.BOOL.equals(f10.getTypeInfo()));
        Assert.assertTrue(ScalarStructFieldType.COMPLEX.equals(f11.getTypeInfo()));
        VectorStructFieldType f12Type = (VectorStructFieldType) f12.getTypeInfo();
        Assert.assertEquals(1, f12Type.getDimensionality());
        Assert.assertEquals(
                "test.symtable.sub1.S2",
                ((StructReferenceFieldType) f12Type.getTypeOfElements()).getReference().getReferencedSymbol().getFullName()
        );
        VectorStructFieldType f13Type = (VectorStructFieldType) f13.getTypeInfo();
        Assert.assertEquals(1, f13Type.getDimensionality());
        Assert.assertEquals(
                "test.symtable.sub2.T1",
                ((StructReferenceFieldType) f13Type.getTypeOfElements()).getReference().getReferencedSymbol().getFullName()
        );
        VectorStructFieldType f14Type = (VectorStructFieldType) f14.getTypeInfo();
        Assert.assertEquals(1, f14Type.getDimensionality());
        Assert.assertEquals(
                "test.symtable.sub3.K2",
                ((StructReferenceFieldType) f14Type.getTypeOfElements()).getReference().getReferencedSymbol().getFullName()
        );
        VectorStructFieldType f15Type = (VectorStructFieldType) f15.getTypeInfo();
        Assert.assertEquals(1, f15Type.getDimensionality());
        Assert.assertEquals(
                "test.symtable.sub2.T5",
                ((StructReferenceFieldType) f15Type.getTypeOfElements()).getReference().getReferencedSymbol().getFullName()
        );
        Assert.assertTrue(ScalarStructFieldType.INTEGRAL.equals(f16.getTypeInfo()));
        VectorStructFieldType f17Type = (VectorStructFieldType) f17.getTypeInfo();
        Assert.assertEquals(10, f17Type.getDimensionality());
        Assert.assertEquals(
                "test.symtable.sub1.S1",
                ((StructReferenceFieldType) f17Type.getTypeOfElements()).getReference().getReferencedSymbol().getFullName()
        );
    }

    @Test
    public void testResolveField1() {
        StructFieldDefinitionSymbol f1 = symTab.<StructFieldDefinitionSymbol>resolve("test.symtable.MyFancyStruct1.f1", StructFieldDefinitionSymbol.KIND).orElse(null);
        Assert.assertNotNull(f1);
        Assert.assertEquals("f1", f1.getName());
        ScalarStructFieldType f1Type = (ScalarStructFieldType) f1.getTypeInfo();
        Assert.assertNotNull(f1Type);
        Assert.assertTrue(ScalarStructFieldType.BOOL.equals(f1Type));
    }

    @Test
    public void testNonExistentReferences() {
        StructSymbol struct = symTab.<StructSymbol>resolve("test.symtable.ErrNonExistentReferences", StructSymbol.KIND).orElse(null);
        Assert.assertNotNull(struct);
        List<StructFieldDefinitionSymbol> fields = new ArrayList<>(struct.getStructFieldDefinitions());
        for (StructFieldDefinitionSymbol f : fields) {
            StructReferenceFieldType fieldType = (StructReferenceFieldType) f.getTypeInfo();
            try {
                fieldType.getReference().getReferencedSymbol();
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
            StructReferenceFieldType typeInfo = (StructReferenceFieldType) field.getTypeInfo();
            Assert.assertNotNull(typeInfo);
            try {
                typeInfo.getReference().getReferencedSymbol();
            } catch (ResolvedSeveralEntriesException e) {
                continue;
            }
            Assert.fail();
        }
    }
}
