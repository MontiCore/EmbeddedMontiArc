/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
