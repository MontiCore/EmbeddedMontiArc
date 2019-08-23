/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.*;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Created by Yannick Deuster on 18.10.17.
 */
public class SymtabTest extends AbstractSymtabTest {
    protected final String resourcePath = "src/test/resources/";

    @Test
    public void testSymtab_Effector() {
        Scope s = createSymTab("view.wbView.WCET1", resourcePath);

        Optional<ViewEffectorSymbol> oEff = s.resolve("view.wbView.WCET1.WeatherBalloonSensors.controlSignalsIndataSaveInternalOut", ViewEffectorSymbol.KIND);
        assertTrue(oEff.isPresent());

        ViewEffectorSymbol eff = oEff.get();
        assertTrue(eff.getSourcePort() != null);
        assertTrue(eff.getTargetPort() != null);

        assertEquals("controlSignalsIn", eff.getSourcePort().getName());
        assertEquals("dataSaveInternalOut", eff.getTargetPort().getName());
    }

    @Test
    public void testSymtab_Component() {
        Scope s = createSymTab("view.wbView.WCET1", resourcePath);

        Optional<ViewComponentSymbol> oCmp = s.resolve("view.wbView.WCET1.WeatherBalloonSensors", ViewComponentSymbol.KIND);
        assertTrue(oCmp.isPresent());

        ViewComponentSymbol cmp = oCmp.get();
        assertTrue(!cmp.getAllIncomingPorts().isEmpty());
        assertTrue(!cmp.getAllOutgoingPorts().isEmpty());
        assertTrue(cmp.hasEffectors("controlSignalsIn"));
    }

    @Test
    public void testSymtab_View() {
        Scope s = createSymTab("view.wbView.WCET1", resourcePath);

        Optional<ViewSymbol> oView = s.resolve("view.wbView.WCET1", ViewSymbol.KIND);
        assertTrue(oView.isPresent());

        ViewSymbol view = oView.get();
        assertTrue(view.getName().equals("WCET1"));
        assertTrue(view.getImports().size() == 2);
        assertTrue(!view.getInnerComponents().isEmpty());
    }

    @Test
    public void testSymtab_Port(){
        Scope s = createSymTab("view.wbView.WCET1", resourcePath);

        Optional<ViewPortSymbol> oPort = s.resolve("view.wbView.WCET1.WeatherBalloonSensors.controlSignalsIn", ViewPortSymbol.KIND);
        assertTrue(oPort.isPresent());

        ViewPortSymbol port = oPort.get();
        assertTrue(port.isIncoming());
        assertTrue(!port.isOutgoing());
    }

    @Test
    public void testSymtab_Connector(){
        Scope s = createSymTab("view.wbView.WCET2", resourcePath);

        Optional<ViewConnectorSymbol> oConn = s.resolve("view.wbView.WCET2.WeatherBalloonSensors.controlSignalsIndataSaveInternalOut", ViewConnectorSymbol.KIND);
        assertTrue(oConn.isPresent());

        ViewConnectorSymbol conn = oConn.get();
        assertTrue(conn.getSource().equals("controlSignalsIn"));
        assertTrue(conn.getTarget().equals("dataSaveInternalOut"));
        assertTrue(conn.getSourcePort().getName().equals("controlSignalsIn"));
        assertTrue(conn.getTargetPort().getName().equals("dataSaveInternalOut"));
    }

    @Test
    public void testSymtab_ComponentInstance() {
        Scope s = createSymTab("view.wbView.WCET2", resourcePath);

        Optional<ViewComponentInstanceSymbol> oCmpInst = s.resolve("view.wbView.WCET2.wbSens", ViewComponentInstanceSymbol.KIND);
        assertTrue(oCmpInst.isPresent());

        ViewComponentInstanceSymbol cmpInst = oCmpInst.get();
        ViewComponentSymbolReference type = cmpInst.getComponentType();
        assertTrue(type.getName().equals("WeatherBalloonSensors"));
    }
}
