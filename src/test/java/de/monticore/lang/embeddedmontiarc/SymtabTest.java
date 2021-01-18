/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcModelLoader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.UnitNumberExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantSIUnit;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import static org.junit.Assert.*;

/**
 * This is a symbol table test.
 *
 */
public class SymtabTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    /**
     * ==============================================
     * Beginning of new EmbeddedMontiArc Tests
     * ==============================================
     *
     */

    //TODO change to assertequals and use jscience for internal stuff

    /**
     * package testing;
     * component SIUnitPortTest{
     * port in (0 m : 0.1 m: 250 m ) distance;
     * <p>
     * }
     */
    @Ignore("fix type, the type is Q and not SIUnitRangesType")
    @Test
    public void testSIUnitRangeSimpleUnit() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitPortTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(0, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getIncomingPort("distance").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();

        assertEquals(0, siUnit.getRange(0).getStartValue().intValue());
        assertEquals(250, siUnit.getRange(0).getEndValue().intValue());
        assertEquals(0.1, siUnit.getRange(0).getStepValue().doubleValue(), 0);
        assertEquals("m", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("m", siUnit.getRange(0).getEndUnit().toString());
        assertEquals("m", siUnit.getRange(0).getStepUnit().toString());

    }

    /**
     * package testing;
     * component SIUnitPortMultiUnitTest{
     * port in (0 m/s : 0.1 m/s: 250 m/s ) speed;
     * <p>
     * }
     */
    @Ignore("fix type, the type is Q and not SIUnitRangesType")
    @Test
    public void testSIUnitRangeMultiUnit() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitPortMultiUnitTest", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(0, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getIncomingPort("speed").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();

        assertEquals(0, siUnit.getRange(0).getStartValue().intValue());
        assertEquals(250, siUnit.getRange(0).getEndValue().intValue());
        assertEquals(0.1, siUnit.getRange(0).getStepValue().doubleValue(), 0);
        assertEquals("m/s", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("m/s", siUnit.getRange(0).getEndUnit().toString());
        assertEquals("m/s", siUnit.getRange(0).getStepUnit().toString());
    }

    /**
     * package testing;
     * component SIUnitRangeDifferentPrefix{
     * ports in [(0km : 0.05m : 10km) (10km : 0.1m : 45 km)] distance;
     * }
     */
    //@Ignore
    @Test
    public void testSIUnitRange2() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitRangeDifferentPrefix", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(0, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getIncomingPort("distance").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();

        assertEquals(0, siUnit.getRange(0).getStartValue().intValue());
        assertEquals(10, siUnit.getRange(0).getEndValue().intValue());
        assertEquals(0.05, siUnit.getRange(0).getStepValue().doubleValue(), 0);
        assertEquals(10, siUnit.getRange(1).getStartValue().intValue());
        assertEquals(45, siUnit.getRange(1).getEndValue().intValue());
        assertEquals(0.1, siUnit.getRange(1).getStepValue().doubleValue(), 0);
        assertEquals("km", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("km", siUnit.getRange(0).getEndUnit().toString());
        assertEquals("km", siUnit.getRange(0).getStepUnit().toString());
        assertEquals("km", siUnit.getRange(1).getStartUnit().toString());
        assertEquals("km", siUnit.getRange(1).getEndUnit().toString());
        assertEquals("m", siUnit.getRange(1).getStepUnit().toString());
    }

    /**
     * package testing;
     * component SIUnitRangeNoLowerBounds{
     * ports in [ -oo km : 45 km)] distance;
     * }
     */
    @Ignore("fix type, the type is Q and not SIUnitRangesType")
    @Test
    public void testSIUnitRangeNoLowerBounds() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitRangeNoLowerBounds", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(0, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getIncomingPort("distance").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();

        assertTrue(siUnit.getRange(0).hasNoLowerLimit());
        assertEquals(45, siUnit.getRange(0).getEndValue().intValue());
        assertEquals("km", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("km", siUnit.getRange(0).getEndUnit().toString());

    }

    /**
     * package testing;
     * component SIUnitRangeNoUpperBounds{
     * ports in (0 m : oo m) distance;
     * }
     */
    @Ignore("fix type, the type is Q and not SIUnitRangesType")
    @Test
    public void testSIUnitRangeNoUpperBounds() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitRangeNoUpperBounds", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(0, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getIncomingPort("distance").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();
        //Log.debug(siUnit.getRange(0).getEndValue());
        assertTrue(siUnit.getRange(0).hasNoUpperLimit());
        assertEquals(Rational.ZERO, siUnit.getRange(0).getStartValue());
        assertEquals("m", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("m", siUnit.getRange(0).getEndUnit().toString());

    }

    /**
     * package testing;
     * component SIUnitRangeConnector{
     * port out [(0 cm: 0.05 cm : 10 cm) (10 cm: 0.1 cm : 45 cm)] distance;
     * connect 0.1 cm -> steering;
     * }
     * }
     */
    //@Ignore
    @Test
    public void testSIUnitRangeConnector() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SIUnitRangeConnector", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(1, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getOutgoingPort("distance").orElse(null);
        assertNotNull(port1);
        assertEquals("SIUnitRangesType", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");
        SIUnitRangesSymbol siUnit = (SIUnitRangesSymbol) port1.getTypeReference().getReferencedSymbol();

        //Log.debug(siUnit.getRange(0).getEndValue().toString(),"asdfasdfa");

        assertEquals(0, siUnit.getRange(0).getStepValue().intValue());
        assertEquals(10, siUnit.getRange(0).getEndValue().intValue());
        assertEquals(0.05, siUnit.getRange(0).getStepValue().doubleValue(), 0.00000001);
        assertEquals(10, siUnit.getRange(1).getStartValue().intValue());
        assertEquals(45, siUnit.getRange(1).getEndValue().intValue());
        assertEquals(0.1, siUnit.getRange(1).getStepValue().doubleValue(), 0.00000001);
        assertEquals("cm", siUnit.getRange(0).getStartUnit().toString());
        assertEquals("cm", siUnit.getRange(0).getEndUnit().toString());
        assertEquals("cm", siUnit.getRange(0).getStepUnit().toString());
        assertEquals("cm", siUnit.getRange(1).getStartUnit().toString());
        assertEquals("cm", siUnit.getRange(1).getEndUnit().toString());
        assertEquals("cm", siUnit.getRange(1).getStepUnit().toString());


        assertEquals(1, cs.getConnectors().size());

        EMAConnectorSymbol emaConnectorSymbol = cs.getConnector("steering").get();
        assertTrue(emaConnectorSymbol.getSourcePort().isConstant());
        EMAPortSymbol cps =  emaConnectorSymbol.getSourcePort();
        assertTrue(cps.getConstantValue().get().isSIUnit());

        EMAConstantSIUnit constantSIUnit = (EMAConstantSIUnit) cps.getConstantValue().get();
        assertEquals(0.1, constantSIUnit.getRational().doubleValue(), 0.0000000001);
        assertEquals("cm", constantSIUnit.getUnit().toString());

    }

    /**
     * package testing;
     * component BooleanConnector{
     * ports out Boolean steering;
     * connect false -> steering;
     * }
     */
    // @Ignore
    @Test
    public void testBooleanConnector() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.BooleanConnector", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(1, cs.getAllIncomingPorts().size());
        assertEquals(1, cs.getAllOutgoingPorts().size());
        EMAPortSymbol port1 = cs.getOutgoingPort("steering").orElse(null);
        assertNotNull(port1);
        assertEquals("B", port1.getTypeReference().getName());

        Log.debug(port1.getTypeReference().getReferencedSymbol().toString() + "", "TypeReference");

        assertEquals(1, cs.getConnectors().size());
        assertTrue(cs.getConnector("steering").isPresent());
        EMAConnectorSymbol emaConnectorSymbol = cs.getConnector("steering").get();
        //check that this is a constant connector
        assertTrue(emaConnectorSymbol.isConstant());
        assertTrue(emaConnectorSymbol.getSourcePort().isConstant());
        EMAPortSymbol cps = emaConnectorSymbol.getSourcePort();

        assertTrue(cps.getConstantValue().get().isBoolean());


        assertEquals(false, cps.getConstantValue().get().getValue());

    }


    /**
     * package testing;
     * <p>
     * component PortArray{
     * port
     * in Boolean lightsIn[5];
     * }
     */
    //@Ignore
    @Test
    public void testPortArray() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.PortArray", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        EMAPortSymbol ps = symTab.<EMAPortSymbol>resolve("testing.PortArray.lightsIn[1]", EMAPortSymbol.KIND).orElse(null);
        assertNotNull("EMAPortSymbol is null", ps);
        Log.debug(ps.getTypeReference().getReferencedSymbol().toString() + "", "TypeName");
//check for all names
        assertEquals(5, cs.getAllIncomingPorts().size());
        assertEquals(5, cs.getAllOutgoingPorts().size());
        for (EMAConnectorSymbol con : cs.getConnectors()) {
            Log.debug(con.toString(), "testPortArray");
        }
    }

    /**
     * package testing;
     * <p>
     * component PortArray{
     * port
     * in Boolean lightsIn[5];
     * }
     */
    @Test
    public void testPortArraySymbol() {
        Scope symTab = createSymTab("src/test/resources");
        EMAPortArraySymbol pas = symTab.<EMAPortArraySymbol>resolve(
                "testing.PortArray.lightsIn", EMAPortArraySymbol.KIND).orElse(null);
        assertNotNull("EMAPortArraySymbol is null", pas);

        assertEquals(5, pas.getDimension());
        assertEquals(5, pas.getConcretePortSymbols().size());
        assertEquals("lightsIn[1]", pas.getConcretePortSymbols().get(0).getName());
        assertEquals("lightsIn[2]", pas.getConcretePortSymbols().get(1).getName());
        assertEquals("lightsIn[3]", pas.getConcretePortSymbols().get(2).getName());
        assertEquals("lightsIn[4]", pas.getConcretePortSymbols().get(3).getName());
        assertEquals("lightsIn[5]", pas.getConcretePortSymbols().get(4).getName());

    }

    /**
     * package testing;
     * component LightControlSwitch{
     * ports
     * in Boolean lightsIn[5],
     * out Boolean lightsOut[5];
     * <p>
     * connect lightsIn[:] -> lightsOut[:];
     * }
     */
    @Test
    public void testPortArrayAutoConnector() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.LightControlSwitch", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(5, cs.getAllIncomingPorts().size());
        for (int i = 1; i <= 5; ++i) {
            EMAPortSymbol port = cs.getIncomingPort("lightsIn[" + i + "]").orElse(null);
            assertEquals(true, port != null);
            assertEquals("B", port.getTypeReference().getName());
        }
        assertEquals(5, cs.getAllOutgoingPorts().size());
        for (int i = 1; i <= 5; ++i) {
            EMAPortSymbol port = cs.getOutgoingPort("lightsOut[" + i + "]").orElse(null);
            assertEquals(true, port != null);
            assertEquals("B", port.getTypeReference().getName());
        }
        assertEquals(5, cs.getConnectors().size());
        for (int i = 1; i <= 5; ++i) {
            EMAConnectorSymbol con = cs.getConnector("lightsOut[" + i + "]").orElse(null);
            assertEquals(true, con != null);
            assertEquals("lightsIn[" + i + "]", con.getSourcePort().getName());
            assertEquals("B", con.getSourcePort().getTypeReference().getName());
            assertEquals("lightsOut[" + i + "]", con.getTargetPort().getName());
            assertEquals("B", con.getTargetPort().getTypeReference().getName());
        }
    }

    /**
     * package testing;
     * component ComponentArray{
     * component PortArray portArray[51];
     * }
     */
    //@Ignore
    @Test
    public void testComponentArray() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.ComponentArray", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);
        //test if amount of SubComponents is correct
        assertEquals(5, cs.getSubComponents().size());
        //test if every SubComponent can be accessed by its corresponding name
        for (int i = 1; i <= 5; ++i) {
            assertEquals(true, cs.getSubComponent("portArray[" + i + "]").isPresent());
            //add type check
        }
    }

    /**
     * package testing;
     * component ControlUnit{
     * ports
     * in Boolean lightsIn[50],
     * out Boolean lightsOut[50];
     * component LightControlSwitch lightControlSwitch[10];
     * <p>
     * connect lightsIn[:] -> lightControlSwitch[:].lightsIn[:];
     * }
     */
    //@Ignore
    @Test
    public void testConnectorArray() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.ControlUnit", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        Log.debug("" + cs.getConnectors().size(), "Connectors");
        assertEquals(50, cs.getConnectors().size());
        EMAConnectorSymbol conn = cs.getConnectors().iterator().next();
        assertEquals("ConnectorSymbols not equal", conn.getComponent(), cs);
        //for loop to check everything
        EMAPortSymbol source = conn.getSourcePort();
        assertNotNull("EMAPortSymbol source is null", source);
        assertTrue("EMAPortSymbol source is notIncoming", source.isIncoming());
        assertEquals("EMAPortSymbol source name is not correct", source.getName(), "lightsIn[1]");
        assertEquals("EMAPortSymbol sourceType is not correct", source.getTypeReference().getName(), "B");
        assertEquals("EMAPortSymbol connector is not correct", source.getComponent(), cs);

        EMAPortSymbol target = conn.getTargetPort();
        assertNotNull("target is null", target);
        assertTrue("target is not incoming", target.isIncoming());
        assertEquals("target name is not correct", target.getName(), "lightsIn[1]");
        assertEquals("target type is not Boolean", target.getTypeReference().getName(), "B");
        assertEquals("testing.LightControlSwitch", target.getComponent().getFullName());
    }

    /**
     * package testing;
     * <p>
     * component ConnectorArraymn{
     * ports
     * in Boolean sIn[5],
     * out Boolean sOut[5];
     * <p>
     * <p>
     * connect sIn[1:3] -> sOut[2:4];
     * <p>
     * }
     */
    //@Ignore
    @Test
    public void testConnectorArraymn() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.ConnectorArraymn", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);

        assertEquals(3, cs.getConnectors().size());
        Iterator iter = cs.getConnectors().iterator();
        EMAConnectorSymbol conn = (EMAConnectorSymbol) iter.next();
        assertEquals("ConnectorSymbols not equal", conn.getComponent(), cs);
        //for loop to check everything
        EMAPortSymbol source = conn.getSourcePort();
        assertNotNull("EMAPortSymbol source is null", source);
        assertTrue("EMAPortSymbol source is notIncoming", source.isIncoming());
        assertEquals("EMAPortSymbol source name is not correct", source.getName(), "sIn[1]");
        assertEquals("EMAPortSymbol sourceType is not correct", source.getTypeReference().getName(), "B");
        assertEquals("EMAPortSymbol connector is not correct", source.getComponent(), cs);

        EMAPortSymbol target = conn.getTargetPort();
        assertNotNull("target is null", target);
        assertTrue("target is not outcoming", target.isOutgoing());
        assertEquals("target name is not correct", target.getName(), "sOut[2]");
        assertEquals("target type is not Boolean", target.getTypeReference().getName(), "B");
        assertEquals("testing.ConnectorArraymn", target.getComponent().getFullName());

        conn = (EMAConnectorSymbol) iter.next();
        assertEquals("ConnectorSymbols not equal", conn.getComponent(), cs);
        //for loop to check everything
        source = conn.getSourcePort();
        assertNotNull("EMAPortSymbol source is null", source);
        assertTrue("EMAPortSymbol source is notIncoming", source.isIncoming());
        assertEquals("EMAPortSymbol source name is not correct2", source.getName(), "sIn[2]");
        assertEquals("EMAPortSymbol sourceType is not correct", source.getTypeReference().getName(), "B");
        assertEquals("EMAPortSymbol connector is not correct", source.getComponent(), cs);

        target = conn.getTargetPort();
        assertNotNull("target is null", target);
        assertTrue("target is not outcoming", target.isOutgoing());
        assertEquals("target name is not correct3", target.getName(), "sOut[3]");
        assertEquals("target type is not Boolean", target.getTypeReference().getName(), "B");
        assertEquals("testing.ConnectorArraymn", target.getComponent().getFullName());


        conn = (EMAConnectorSymbol) iter.next();
        assertEquals("ConnectorSymbols not equal", conn.getComponent(), cs);
        //for loop to check everything
        source = conn.getSourcePort();
        assertNotNull("EMAPortSymbol source is null", source);
        assertTrue("EMAPortSymbol source is notIncoming", source.isIncoming());
        assertEquals("EMAPortSymbol source name is not correct4", source.getName(), "sIn[3]");
        assertEquals("EMAPortSymbol sourceType is not correct", source.getTypeReference().getName(), "B");
        assertEquals("EMAPortSymbol connector is not correct", source.getComponent(), cs);

        target = conn.getTargetPort();
        assertNotNull("target is null", target);
        assertTrue("target is not outcoming", target.isOutgoing());
        assertEquals("target name is not correct5", target.getName(), "sOut[4]");
        assertEquals("target type is not Boolean", target.getTypeReference().getName(), "B");
        assertEquals("testing.ConnectorArraymn", target.getComponent().getFullName());
    }

    /**
     * package testing;
     * <p>
     * component WrongPortNumber{
     * port
     * in Boolean sIn[5],
     * out Boolean sOut[5];
     * <p>
     * connect sIn[1:2] -> sOut[2:5];
     * <p>
     * }
     */
    @Test
    public void testWrongPortNumber() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.WrongPortNumber", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull("EMAComponentSymbol is null", cs);
        //TODO test for [n:m]
    }


    @Test
    public void testTypeGenerics() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.BasicTypeInstance", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);
        EMAComponentInstantiationSymbol csInner = symTab.<EMAComponentInstantiationSymbol>resolve("testing.BasicTypeInstance.b1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        assertNotNull(csInner);

        assertEquals("b1", csInner.getName());
    }


    @Test
    public void testTypeGenericsInstanciation() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.BasicTypeInstance", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);
        EMAComponentInstantiationSymbol csInner = symTab.<EMAComponentInstantiationSymbol>resolve("testing.BasicTypeInstance.b1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        assertNotNull(csInner);

        assertEquals("b1", csInner.getName());
    }

    /**
     * package testing;
     * <p>
     * component BasicResolutionInstance {
     * port
     * in Boolean in1[5],
     * out Boolean out1;
     * <p>
     * instance BasicResolution<5> br1;
     * <p>
     * connect in1[1]->br1[1];
     * connect in1[2]->br1[2];
     * connect in1[3]->br1[3];
     * connect in1[4]->br1[4];
     * connect in1[5]->br1[5];
     * <p>
     * }
     */
    @Test
    public void testTypeVariableGenericsInstanciation() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol cs = symTab.<EMAComponentInstanceSymbol>resolve("testing.basicResolutionInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(cs);
        //EMAComponentInstantiationSymbol csInner = symTab.<EMAComponentInstantiationSymbol>resolve("testing.BasicResolutionInstance.br1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        //assertNotNull(csInner);

        Log.debug(cs.getSubComponents().iterator().next().getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol() + "", "Expanded:");
        Log.debug(cs.getSubComponents().iterator().next().getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols().get(0).getNameToResolve(), "Name to Resolve:");


        ResolutionDeclarationSymbol jt = cs.getSubComponents().iterator().next().getComponentType().getReferencedSymbol().getResolutionDeclarationSymbol("n").get();
        assertNotNull(jt);
        assertEquals(6, ((ASTUnitNumberResolution) jt.getASTResolution()).getNumber().get().intValue());
        assertEquals("br1", cs.getSubComponents().iterator().next().getName());
        assertEquals(6, cs.getSubComponents().iterator().next().getComponentType().getIncomingPorts().size());

    }


    @Test
    public void testTypeVariableGenericsInstanciationDefault() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.BasicResolutionDefaultInstance", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);
        EMAComponentInstantiationSymbol csInner = symTab.<EMAComponentInstantiationSymbol>resolve("testing.BasicResolutionDefaultInstance.br1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        assertNotNull(csInner);

        Log.debug(csInner.getFullName() + " " + csInner.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol(), "Amount ResolutionDeclarationSymbols :");


        ResolutionDeclarationSymbol jt = csInner.getComponentType().getResolutionDeclarationSymbol("n").get();
        assertNotNull(jt);
        assertEquals(2, ((ASTUnitNumberResolution) jt.getASTResolution()).getNumber().get().intValue());
        assertEquals("br1", csInner.getName());
        assertEquals(2, csInner.getComponentType().getIncomingPorts().size());


        ResolutionDeclarationSymbol jk = csInner.getComponentType().getResolutionDeclarationSymbol("k").get();
        assertNotNull(jk);
        assertEquals(1, ((ASTUnitNumberResolution) jk.getASTResolution()).getNumber().get().intValue());

    }

    @Test
    public void testTypeVariableGenericsInstanciation2() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.BasicResolutionInstance", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);
        EMAComponentInstantiationSymbol csInner = symTab.<EMAComponentInstantiationSymbol>resolve("testing.BasicResolutionDefaultInstance.br1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        assertNotNull(csInner);

        Log.debug(csInner.getFullName() + " " + csInner.getComponentType().getReferencedSymbol().howManyResolutionDeclarationSymbol(), "Amount ResolutionDeclarationSymbols :");


        ResolutionDeclarationSymbol jt = csInner.getComponentType().getResolutionDeclarationSymbol("n").get();
        assertNotNull(jt);
        assertEquals(6, ((ASTUnitNumberResolution) jt.getASTResolution()).getNumber().get().intValue());
        assertEquals("br1", csInner.getName());
        assertEquals(6, csInner.getComponentType().getIncomingPorts().size());


        ResolutionDeclarationSymbol jk = csInner.getComponentType().getResolutionDeclarationSymbol("k").get();
        assertNotNull(jk);
        assertEquals(3, InstanceInformation.getInstanceNumberFromASTSubComponent(csInner.getInstanceInformation().get().getASTSubComponent(), 1));

    }


    @Test
    public void testSubComponentDefinitionInstanciation() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("testing.SubComponent", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);


    }

    @Test
    public void testBumperBotEmergency() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cs = symTab.<EMAComponentSymbol>resolve("bumperBotEmergency.BumperBotEmergency", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(cs);


    }

    @Test
    public void testComponentGenericsSameComponent() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol inst = symTab.<EMAComponentSymbol>resolve(
                "testing.BasicParameterInstance", EMAComponentSymbol.KIND).orElse(null);

        assertNotNull(inst);
        System.out.println(inst.getSubComponents().iterator().next().getComponentType());
        for (ASTExpression astExpression : inst.getSubComponents().iterator().next().getComponentType().getArguments()) {
            Log.info(astExpression.toString(), "info:");
        }
        assertEquals(2, inst.getSubComponents().size());
        Iterator<EMAComponentInstantiationSymbol> iterator = inst.getSubComponents().iterator();
        UnitNumberExpressionSymbol symbol1 = (UnitNumberExpressionSymbol) iterator.next().getComponentType().getArguments().get(0).getSymbolOpt().get();
        UnitNumberExpressionSymbol symbol2 = (UnitNumberExpressionSymbol) iterator.next().getComponentType().getArguments().get(0).getSymbolOpt().get();
        assertEquals("5", symbol1.getTextualRepresentation());
        assertEquals("1", symbol2.getTextualRepresentation());
    }

    @Test
    public void testBasicInputPortWrapped() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol cs = symTab.<EMAComponentInstanceSymbol>resolve("testing.basicInputPortWrapped", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(cs);

        assertEquals(2, cs.getConnectorInstances().size());

    }


    @Test
    public void testColonTest() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol cs = symTab.<EMAComponentInstanceSymbol>resolve("test.a.colonTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(cs);

        assertEquals(8, cs.getConnectorInstances().size());
        Iterator<EMAConnectorInstanceSymbol> connectorSymbolIter = cs.getConnectorInstances().iterator();

        EMAConnectorSymbol cur = connectorSymbolIter.next();
        assertEquals("pass.pout1[1]", cur.getSource());
        assertEquals("out1[1]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("pass.pout1[2]", cur.getSource());
        assertEquals("out1[2]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("pass.pout1[3]", cur.getSource());
        assertEquals("out1[3]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("pass.pout1[4]", cur.getSource());
        assertEquals("out1[4]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("in1[1]", cur.getSource());
        assertEquals("pass.pin1[1]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("in1[2]", cur.getSource());
        assertEquals("pass.pin1[2]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("in1[1]", cur.getSource());
        assertEquals("pass.pin1[3]", cur.getTarget());
        cur = connectorSymbolIter.next();
        assertEquals("in1[2]", cur.getSource());
        assertEquals("pass.pin1[4]", cur.getTarget());

    }

    @Test
    public void testComponentWithStructPorts() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol cmp = symTab.<EMAComponentSymbol>resolve(
                "testing.ComponentWithStructPorts",
                EMAComponentSymbol.KIND
        ).orElse(null);
        assertNotNull(cmp);
        List<EMAPortSymbol> inPorts = new ArrayList<>(cmp.getAllIncomingPorts());
        assertEquals(1, inPorts.size());
        EMAPortSymbol in1 = cmp.getIncomingPort("in1").orElse(null);
        assertNotNull(in1);
        MCTypeSymbol s1 = in1.getTypeReference().getReferencedSymbol();
        assertNotNull(s1);
        assertEquals("structures.S1", s1.getFullName());
        assertTrue(s1 instanceof StructSymbol);
        List<EMAPortSymbol> outPorts = new ArrayList<>(cmp.getAllOutgoingPorts());
        assertEquals(1, outPorts.size());
        EMAPortSymbol out1 = cmp.getOutgoingPort("out1").orElse(null);
        assertNotNull(out1);
        MCTypeSymbol s2 = out1.getTypeReference().getReferencedSymbol();
        assertEquals("structures.S2", s2.getFullName());
        assertTrue(s2 instanceof StructSymbol);
    }

    @Test
    public void testSymtabFAS() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol instance = symTab.<EMAComponentInstanceSymbol>resolve("fas.demo_fas_Fkt_m.fAS", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(instance);
    }

    @Test
    public void testInstantiations() {
        EmbeddedMontiArcModelLoader loader = new EmbeddedMontiArcModelLoader();
        Scope symTab = loader.createSymTabFromMainTxt("src/test/resources/symtab/instantiations/main.txt");

        EMAComponentSymbol emaComponentSymbol = loader.loadComponentFromMainTxt("src/test/resources/symtab/instantiations/main.txt");
        assertNotNull(emaComponentSymbol);
        Scope componentScope = emaComponentSymbol.getSpannedScope();
        EMAPortSymbol portSymbol = componentScope.<EMAPortSymbol>resolve("top_in_1", EMAPortSymbol.KIND).orElse(null);
        assertNotNull(portSymbol);
        EMAConnectorSymbol connectorSymbol = componentScope.<EMAConnectorSymbol>resolve("top_out_1", EMAConnectorSymbol.KIND).orElse(null);
        assertNotNull(connectorSymbol);
        EMAComponentInstantiationSymbol instantiationSymbol = componentScope.<EMAComponentInstantiationSymbol>resolve("sub_1", EMAComponentInstantiationSymbol.KIND).orElse(null);
        assertNotNull(instantiationSymbol);


        EMAComponentInstanceSymbol instanceSymbol = symTab.<EMAComponentInstanceSymbol>resolve("symtab.instantiations.top_1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(instanceSymbol);
        Scope instanceScope = instanceSymbol.getSpannedScope();
        EMAPortInstanceSymbol portInstanceSymbol = instanceScope.<EMAPortInstanceSymbol>resolve("top_in_1", EMAPortInstanceSymbol.KIND).orElse(null);
        assertNotNull(portInstanceSymbol);
        EMAConnectorInstanceSymbol connectorInstanceSymbol = instanceScope.<EMAConnectorInstanceSymbol>resolve("top_out_1", EMAConnectorInstanceSymbol.KIND).orElse(null);
        assertNotNull(connectorInstanceSymbol);

        EMAComponentInstanceSymbol instanceSymbol2 = componentScope.<EMAComponentInstanceSymbol>resolve("symtab.instantiations.top_1.sub_1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(instanceSymbol2);
        Scope instanceScope2 = instanceSymbol2.getSpannedScope();
        EMAPortInstanceSymbol portInstanceSymbol2 = instanceScope2.<EMAPortInstanceSymbol>resolve("sub_in_1", EMAPortInstanceSymbol.KIND).orElse(null);
        assertNotNull(portInstanceSymbol2);


        EMAComponentInstanceSymbol instanceSymbol3 = symTab.<EMAComponentInstanceSymbol>resolve("symtab.instantiations.top_1.sub_1.sub_2a", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(instanceSymbol3);

        EMAPortInstanceSymbol portInstanceSymbol3 = symTab.<EMAPortInstanceSymbol>resolve("symtab.instantiations.top_1.sub_1.sub_2a.sub_in_1", EMAPortInstanceSymbol.KIND).orElse(null);
        assertNotNull(portInstanceSymbol3);
    }


    @Test
    public void starConnectorsTest() {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve("testing.StarConnector", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(component);

        Collection<EMAConnectorSymbol> connectors = component.getConnectors();
        assertEquals(5, connectors.size());

        EMAConnectorSymbol connector = component.getConnector("a.in1").orElse(null);
        assertNotNull(connector);
        assertEquals("in1", connector.getSource());
        assertEquals("a.in1", connector.getTarget());
        assertTrue(connectors.contains(connector));

        connector = component.getConnector("a.in2").orElse(null);
        assertNotNull(connector);
        assertEquals("in2", connector.getSource());
        assertEquals("a.in2", connector.getTarget());
        assertTrue(connectors.contains(connector));

        connector = component.getConnector("out1").orElse(null);
        assertNotNull(connector);
        assertEquals("a.out1", connector.getSource());
        assertEquals("out1", connector.getTarget());
        assertTrue(connectors.contains(connector));

        connector = component.getConnector("out2").orElse(null);
        assertNotNull(connector);
        assertEquals("a.out2", connector.getSource());
        assertEquals("out2", connector.getTarget());
        assertTrue(connectors.contains(connector));

        connector = component.getConnector("#").orElse(null);
        assertNotNull(connector);
        assertEquals("in3", connector.getSource());
        assertEquals("#", connector.getTarget());
        assertTrue(connectors.contains(connector));

    }

    @Test
    public void testBasicGenericArrayInstance(){
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol componentInstanceSymbol = symTab.<EMAComponentInstanceSymbol>resolve("test.basicGenericArrayInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        for(int i = 1; i <= 6; i++){
            assertTrue(componentInstanceSymbol.getPortInstance("aVal1["+i+"]").isPresent());
            assertTrue(componentInstanceSymbol.getPortInstance("aValOut["+i+"]").isPresent());
        }

        EMAComponentInstanceSymbol subComp1 = componentInstanceSymbol.getSubComponent("basicGenericArraySize1").orElse(null);
        EMAComponentInstanceSymbol subComp2 = componentInstanceSymbol.getSubComponent("basicGenericArraySize2").orElse(null);

        assertNotNull(subComp1);
        assertNotNull(subComp2);

        for(int i = 1; i <= 3; i++) {
            assertTrue("basicGenericArraySize1.val1["+i+"] not found", subComp1.getPortInstance("val1["+i+"]").isPresent());
            assertTrue("basicGenericArraySize2.val1["+i+"] not found", subComp2.getPortInstance("val1["+i+"]").isPresent());
            assertTrue("basicGenericArraySize1.valOut["+i+"] not found",subComp1.getPortInstance("valOut["+i+"]").isPresent());
            assertTrue("basicGenericArraySize2.valOut["+i+"] not found",subComp2.getPortInstance("valOut["+i+"]").isPresent());
        }

    }

}

