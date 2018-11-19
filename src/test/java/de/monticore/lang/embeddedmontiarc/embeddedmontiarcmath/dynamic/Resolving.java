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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.dynamic;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.AbstractSymtabTest;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static org.junit.Assert.assertNotNull;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class Resolving extends AbstractSymtabTest {

    @Test
    public void test_00(){
        Scope symTab = createSymTab("src/test/resources/emad");


        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(
                "test00.TrueTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(comp);
        Log.info(comp.toString(), "Resolvin.Test00");

        EMADynamicComponentInstanceSymbol inst = symTab.<EMADynamicComponentInstanceSymbol>resolve(
                "test00.trueTest", EMADynamicComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);

    }

    @Test
    public void test_01(){
        Scope symTab = createSymTab("src/test/resources/emad");


        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(
                "test01.TrueTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(comp);
        Log.info(comp.toString(), "Resolvin.Test01");

        EMADynamicComponentInstanceSymbol inst = symTab.<EMADynamicComponentInstanceSymbol>resolve(
                "test01.trueTest", EMADynamicComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);

    }

    @Test
    public void test_02(){
        Scope symTab = createSymTab("src/test/resources/emad");


        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(
                "test02.ConnectedTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(comp);
        Log.info(comp.toString(), "Resolvin.Test02");

        EMADynamicComponentInstanceSymbol inst = symTab.<EMADynamicComponentInstanceSymbol>resolve(
                "test02.connectedTest", EMADynamicComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
    }

    @Test
    public void test_03(){
        Scope symTab = createSymTab("src/test/resources/emam");


        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(
                "test.GenericsTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(comp);

        Log.info(comp.toString(), "Resolving.Test03");


        EMADynamicComponentInstanceSymbol inst = symTab.<EMADynamicComponentInstanceSymbol>resolve(
                "test.genericsTest", EMADynamicComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Resolvin.Test03");
    }

}
