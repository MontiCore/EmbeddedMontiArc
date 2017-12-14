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
package de.monticore.lang.monticar.enumlang;

import de.monticore.lang.monticar.enumlang._symboltable.EnumConstantDeclarationSymbol;
import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.symboltable.Scope;
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
    public void testEnumResolution() {
        EnumDeclarationSymbol s = symTab.<EnumDeclarationSymbol>resolve(
                "test.symtable.AnEnum1",
                EnumDeclarationSymbol.KIND
        ).orElse(null);
        Assert.assertNotNull(s);
        List<EnumConstantDeclarationSymbol> enumConstants = new ArrayList<>(s.getEnumConstants());
        Assert.assertEquals(3, enumConstants.size());
        Assert.assertEquals("A_CONSTANT_1", enumConstants.get(0).getName());
        Assert.assertEquals("A_CONSTANT_2", enumConstants.get(1).getName());
        Assert.assertEquals("A_CONSTANT_3", enumConstants.get(2).getName());
    }

    @Test
    public void testEnumConstantResolution() {
        EnumConstantDeclarationSymbol s = symTab.<EnumConstantDeclarationSymbol>resolve(
                "test.symtable.TrafficLight.YELLOW",
                EnumConstantDeclarationSymbol.KIND
        ).orElse(null);
        Assert.assertNotNull(s);
        Assert.assertEquals("YELLOW", s.getName());
        Assert.assertEquals("test.symtable.TrafficLight.YELLOW", s.getFullName());
    }
}
