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
package de.monticore.lang.embeddedmontiarc;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.*;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstanceInformation;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.util.Iterator;

import static org.junit.Assert.*;

/**
 * Tests for toString methods of EmbeddedMontiArc symbols.
 *
 * @author Michael von Wenckstern
 */
public class ExpandedComponentInstanceTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testFAS() throws Exception {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "fas.demo_fas_Fkt_m.fAS", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
    }


    @Test
    public void testComponentSub2() throws Exception {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "a.sub2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        System.out.println(inst);

        assertEquals(inst.getPortInstanceList().size(), 3);
        assertTrue(inst.getPortInstance("in1[1]").isPresent()); // from a.Sub2
        assertTrue(inst.getPortInstance("out1").isPresent()); // from a.Sub2

        for (EMAConnectorSymbol con : inst.getConnectorInstances()) {
            Log.debug(con.toString(), "testComponentSub2");
        }
    }

    // @Ignore
    @Test
    public void testComponentSub2Sanity() throws Exception {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol inst = symTab.<EMAComponentSymbol>resolve(
                "a.Sub2", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(inst);
        System.out.println(inst);

        assertEquals(inst.getPortsList().size(), 3);
        //assertTrue(inst.getPortInstance("in1").isPresent()); // from a.Sub2
        //assertTrue(inst.getPortInstance("out1").isPresent()); // from a.Sub2

        for (EMAConnectorSymbol con : inst.getConnectors()) {
            Log.debug(con.toString(), "testComponentSub2");
        }
    }
/* TODO add more tests*/

    @Ignore("fix type, the type is Q and not RangeType")
    @Test
    public void testSubGenericInstance() throws Exception {
        Scope symTab = createSymTab("src/test/resources/symtab");
        EMAComponentSymbol emaComponentSymbol = symTab.<EMAComponentSymbol>resolve("generics.SubGeneric", EMAComponentSymbol.KIND).orElse(null);

        assertNotNull(emaComponentSymbol);
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "generics.subGenericInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);
        // test whether T is replaced by Integer
        inst.getPortInstanceList().stream().forEachOrdered(p -> assertEquals(p.getTypeReference().getName(), "Integer"));

        EMAComponentInstanceSymbol inst2 = symTab.<EMAComponentInstanceSymbol>resolve(
                "generics.superGenericCompInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst2);
        System.out.println(inst2);
        // test whether T is replaced by Integer
        assertEquals("RangeType", inst2.getSubComponent("sgc").get().getPortInstance("tIn").get().getTypeReference().getName());
        assertEquals("RangeType", inst2.getSubComponent("sgc").get().getPortInstance("tOut").get().getTypeReference().getName());

        assertEquals("B", inst2.getSubComponent("sgc2").get().getPortInstance("tIn").get().getTypeReference().getName());
        assertEquals("RangeType", inst2.getSubComponent("sgc2").get().getPortInstance("tOut").get().getTypeReference().getName());
    }

    @Test
    public void testConnectorInstancing() {
        ConstantPortHelper.resetLastID();
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.connectorInstancing", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println("inst: " + inst.toString());

        assertEquals(3, inst.getConnectorInstances().size());
        // Todo: make ConstantPorts Instances
        //ConstantPortHelper portSymbol = (ConstantPortHelper) inst.getPortInstance("CONSTANTPORT1").get();
    }

    @Test
    public void testGenericA() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.genericAInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        EMAComponentSymbolReference symbol = inst.getSubComponents().iterator().next().getComponentType();
        Log.info(symbol.getReferencedSymbol().toString(), "component:");
        Log.debug(inst.getSubComponents().iterator().next().getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Expanded:");

        assertNotNull(inst);
        System.out.println(inst);
        Log.debug(inst.getUnitNumberResolutionSubComponents("n") + "", "Expanded:");

    }


    @Test
    public void testSubGenericValueInstance() throws Exception {
        Scope symTab = createSymTab("src/test/resources/symtab");
        EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve("generics.SubGenericValue", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(comp);
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "generics.subGenericValueInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);
        // test whether T is replaced by Integer
    /*    inst.getPortInstanceList().stream().forEachOrdered(p -> assertEquals(p.getTypeReference().getName(), "Integer"));

        EMAComponentInstanceSymbol inst2 = symTab.<EMAComponentInstanceSymbol>resolve(
                "generics.superGenericCompInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst2);
        System.out.println(inst2);
        // test whether T is replaced by Integer
        assertEquals("RangeType", inst2.getSubComponent("sgc").get().getPortInstance("tIn").get().getTypeReference().getName());
        assertEquals("RangeType", inst2.getSubComponent("sgc").get().getPortInstance("tOut").get().getTypeReference().getName());

        assertEquals("UnitNumberResolution", inst2.getSubComponent("sgc2").get().getPortInstance("tIn").get().getTypeReference().getName());
        assertEquals("RangeType", inst2.getSubComponent("sgc2").get().getPortInstance("tOut").get().getTypeReference().getName());
    */
    }

    @Test
    public void testBasicParameterInstance() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.basicParameterInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);
        assertEquals(2, inst.getSubComponents().size());
        assertEquals(1, inst.getSubComponents().iterator().next().getParameters().size());
        for (ASTExpression astExpression : inst.getSubComponents().iterator().next().getArguments()) {
            Log.info(astExpression.toString(), "info:");
        }
        Iterator<EMAComponentInstanceSymbol> iterator = inst.getSubComponents().iterator();
        UnitNumberExpressionSymbol symbol1 = (UnitNumberExpressionSymbol) iterator.next().getArguments().get(0).getSymbolOpt().get();
        UnitNumberExpressionSymbol symbol2 = (UnitNumberExpressionSymbol) iterator.next().getArguments().get(0).getSymbolOpt().get();
        assertEquals("5", symbol1.getTextualRepresentation());
        assertEquals("1", symbol2.getTextualRepresentation());
    }

    @Test
    public void testAdaptableParameterInstance() {
        Scope symtab = createSymTab("src/test/resources");

        EMAComponentInstanceSymbol comp = symtab.<EMAComponentInstanceSymbol>resolve("testing.adaptableParameterInstance",EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        EMAComponentInstanceSymbol subInst1 = comp.getSubComponent("adaptableParameter").orElse(null);
        assertNotNull(subInst1);

        EMAPortSymbol configPort = subInst1.getIncomingPortInstance("param1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());

        assertNull(subInst1.getIncomingPortInstance("param2").orElse(null));
    }

    @Test
    public void testConfigPort(){
        Scope symtab = createSymTab("src/test/resources");

        EMAComponentInstanceSymbol comp = symtab.<EMAComponentInstanceSymbol>resolve("testing.configPort",EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(comp);

        EMAPortSymbol configPort = comp.getIncomingPortInstance("in1").orElse(null);
        assertNotNull(configPort);
        assertTrue(configPort.isConfig());

        EMAPortSymbol nonConfigPort = comp.getIncomingPortInstance("in2").orElse(null);
        assertNotNull(nonConfigPort);
        assertFalse(nonConfigPort.isConfig());
    }

    @Test
    public void testExtensionMechanism1() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "a.superCompExtension", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);

    }

    @Test
    public void testExtensionMechanism2() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "a.superCompGenericExtension", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);

    }

    //Currently not working
    @Ignore
    @Test
    public void testExtensionMechanism3() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "a.superCompGenericGenericExtensionInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst);

    }

    @Test
    public void testTypeVariableGenericsInstanciation2() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol csInner = symTab.<EMAComponentInstanceSymbol>resolve("testing.basicResolutionInstance.br1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(csInner);

        Log.debug(csInner.getFullName() + " " + csInner.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol(), "Amount ResolutionDeclarationSymbols :");


        ResolutionDeclarationSymbol jt = csInner.getResolutionDeclarationSymbol("n");
        assertNotNull(jt);
        //This works for all generic indices
        int result = InstanceInformation.getInstanceNumberFromASTSubComponent(csInner.getInstanceInformation().get().getASTSubComponent(), 0);
        assertEquals(6, result);
        //not working in expandedcomponentInstanceSymbols, use InstanceInformation.getInstanceNumber... as seen above
        //assertEquals(6, ((ASTUnitNumberResolution) jt.getASTResolution()).getNumber().get().intValue());
        assertEquals("br1", csInner.getName());
        assertEquals(6, csInner.getComponentType().getIncomingPorts().size());
        result = InstanceInformation.getInstanceNumberFromASTSubComponent(csInner.getInstanceInformation().get().getASTSubComponent(), 1);
        assertEquals(3, result);
    }

    @Test
    public void testConnectorCorrectness() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subComponentConnector", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);

        testConnectorCorrectnessForComponent(inst);
    }


    @Test
    public void testConnectorCorrectness2() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subComponentConnector2", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(inst);

        assertEquals(2, inst.getConnectorInstances().size());

        Iterator<EMAConnectorInstanceSymbol> iter = inst.getConnectorInstances().iterator();
        EMAConnectorInstanceSymbol cs = iter.next();

        assertEquals("a1.out1", cs.getSource());
        assertEquals("out1", cs.getTarget());
        assertEquals("testing.subComponentConnector2.a1.out1", cs.getSourcePort().getFullName());
        assertEquals("testing.subComponentConnector2.out1", cs.getTargetPort().getFullName());

        cs = iter.next();

        assertEquals("in1", cs.getSource());
        assertEquals("a1.in1", cs.getTarget());
        assertEquals("testing.subComponentConnector2.in1", cs.getSourcePort().getFullName());
        assertEquals("testing.subComponentConnector2.a1.in1", cs.getTargetPort().getFullName());

        assertEquals(1, inst.getSubComponents().size());

        EMAComponentInstanceSymbol inst2 = inst.getSubComponents().iterator().next();

        assertEquals(2, inst2.getConnectorInstances().size());
        iter = inst2.getConnectorInstances().iterator();
        cs = iter.next();
        assertEquals("a1.out1", cs.getSource());
        assertEquals("out1", cs.getTarget());
        assertEquals("testing.subComponentConnector2.a1.a1.out1", cs.getSourcePort().getFullName());
        assertEquals("testing.subComponentConnector2.a1.out1", cs.getTargetPort().getFullName());

        cs = iter.next();
        assertEquals("in1", cs.getSource());
        assertEquals("a1.in1", cs.getTarget());
        assertEquals("testing.subComponentConnector2.a1.in1", cs.getSourcePort().getFullName());
        assertEquals("testing.subComponentConnector2.a1.a1.in1", cs.getTargetPort().getFullName());

    }

    private void testConnectorCorrectnessForComponent(EMAComponentInstanceSymbol inst) {
        inst.getConnectorInstances().forEach(connectorSymbol -> {
            assertNotNull(connectorSymbol.getSourcePort());
            assertNotNull(connectorSymbol.getTargetPort());

            EMAPortSymbol sourcePort = connectorSymbol.getSourcePort();
            EMAPortSymbol targetPort = connectorSymbol.getTargetPort();

            System.out.println("source: " + sourcePort.getFullName());
            System.out.println("target: " + targetPort.getFullName() + "\n");

            assertNotEquals(sourcePort.getFullName(), targetPort.getFullName());
        });

        inst.getSubComponents().forEach(this::testConnectorCorrectnessForComponent);
    }

/*
  @Test
  public void testGenericInstance() throws Exception {
    Scope symTab = createSymTab("src/test/resources/arc/symtab");
    EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
        "generics.genericInstance", EMAComponentInstanceSymbol.KIND).orElse(null);

    assertNotNull(inst);
    System.out.println(inst);

    //<editor-fold desc="How to derive types through generics">
    /*
    component GenericInstance {
      component Generic<T extends Number> {
        ports in T in1,
              in T in2,
              out T out1;

        component SuperGenericComparableComp2<String, T> sc1;
        component SuperGenericComparableComp2<Integer, T> sc2;
      }

      component Generic<Double> gDouble;
      component Generic<Integer> gInteger;
    }

    component SuperGenericComparableComp2<K, T extends Comparable<T>> {

        port
            in T tIn,
            out K tOut;
    }

    ==>
    component GenericInstance {
      component Generic<T=Double> gDouble {
        ports in Double in1,
              in Double in2,
              out Double out1;

        component SuperGenericComparableComp2<K=String, T=Double> sc1 {
          port
            in Double tIn,
            out String tOut;
        }

        component SuperGenericComparableComp2<K=Integer, T=Double> sc2 {
          port
            in Double tIn,
            out Integer tOut;
        }
      }

      component Generic<T=Integer> gInteger {
        ports in Integer in1,
              in Integer in2,
              out Integer out1;

        component SuperGenericComparableComp2<K=String, T=Integer> sc1 {
          port
            in Integer tIn,
            out String tOut;
        }

        component SuperGenericComparableComp2<K=Integer, T=Integer> sc2 {
          port
            in Integer tIn,
            out Integer tOut;
        }
      }
     */
    //</editor-fold>
 /*
    assertEquals(inst.getSubComponent("gDouble").get().getSubComponent("sc1")
        .get().getPortInstance("tIn").get().getTypeReference().getName(), "Double");
    assertEquals(inst.getSubComponent("gDouble").get().getSubComponent("sc1")
        .get().getPortInstance("tOut").get().getTypeReference().getName(), "String");

    assertEquals(inst.getSubComponent("gDouble").get().getSubComponent("sc2")
        .get().getPortInstance("tIn").get().getTypeReference().getName(), "Double");
    assertEquals(inst.getSubComponent("gDouble").get().getSubComponent("sc2")
        .get().getPortInstance("tOut").get().getTypeReference().getName(), "Integer");

    assertEquals(inst.getSubComponent("gInteger").get().getSubComponent("sc1")
        .get().getPortInstance("tIn").get().getTypeReference().getName(), "Integer");
    assertEquals(inst.getSubComponent("gInteger").get().getSubComponent("sc1")
        .get().getPortInstance("tOut").get().getTypeReference().getName(), "String");

    assertEquals(inst.getSubComponent("gInteger").get().getSubComponent("sc2")
        .get().getPortInstance("tIn").get().getTypeReference().getName(), "Integer");
    assertEquals(inst.getSubComponent("gInteger").get().getSubComponent("sc2")
        .get().getPortInstance("tOut").get().getTypeReference().getName(), "Integer");
  }

  @Test
  public void testGenericExtension() throws Exception {
    Scope symTab = createSymTab("src/test/resources/arc/symtab");
    EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
        "generics.baseClassGenerics", EMAComponentInstanceSymbol.KIND).orElse(null);

    assertNotNull(inst);
    System.out.println(inst);

    assertEquals(inst.getPortInstance("boolIn").get().getTypeReference().getName(), "Boolean");
    assertEquals(inst.getPortInstance("intOut").get().getTypeReference().getName(), "Integer");
    assertEquals(inst.getPortInstance("sIn1").get().getTypeReference().getName(), "String"); // test if T is replaced by String
    assertEquals(inst.getPortInstance("sIn2").get().getTypeReference().getName(), "String");
    assertEquals(inst.getPortInstance("sOut").get().getTypeReference().getName(), "String");
  }

  @Test
  public void testLoadingInstancePort() throws Exception {
    Scope symTab = createSymTab("src/test/resources/arc/symtab");
    EMAPortSymbol port = symTab.<EMAPortSymbol>resolve(
        "a.sub1.cComp.in1", EMAPortSymbol.KIND).orElse(null);
    assertNotNull(port);
    System.out.println(port);
  }

  @Test
  public void testFAS() throws Exception {
    Scope symTab = createSymTab("src/test/resources/fas");
    EMAComponentSymbol cmp = symTab.<EMAComponentSymbol>resolve(
        "DEMO_FAS.DEMO_FAS.DEMO_FAS_Funktion.CC_On_Off", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(cmp);
    EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
        "DEMO_FAS.DEMO_FAS.DEMO_FAS_Funktion.cC_On_Off", EMAComponentInstanceSymbol.KIND).orElse(null);
    assertNotNull(inst);

    symTab = createSymTab("src/test/resources/fas");
    cmp = symTab.<EMAComponentSymbol>resolve(
        "DEMO_FAS.DEMO_FAS.DEMO_FAS_Funktion.Limiter", EMAComponentSymbol.KIND).orElse(null);
    assertNotNull(cmp);
    inst = symTab.<EMAComponentInstanceSymbol>resolve(
        "DEMO_FAS.DEMO_FAS.DEMO_FAS_Funktion.limiter", EMAComponentInstanceSymbol.KIND).orElse(null);
    assertNotNull(inst);
  }
  */
}
