/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.*;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.nio.file.Paths;
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

        Optional<EffectorSymbol> oEff = s.resolve("view.wbView.WCET1.WeatherBalloonSensors.controlSignalsIndataSaveInternalOut", EffectorSymbol.KIND);
        assertTrue(oEff.isPresent());

        EffectorSymbol eff = oEff.get();
        assertTrue(eff.getSourcePort() != null);
        assertTrue(eff.getTargetPort() != null);

        assertEquals("controlSignalsIn", eff.getSourcePort().getName());
        assertEquals("dataSaveInternalOut", eff.getTargetPort().getName());
    }

    @Test
    public void testSymtab_Component() {
        Scope s = createSymTab("view.wbView.WCET1", resourcePath);

        Optional<ComponentSymbol> oCmp = s.resolve("view.wbView.WCET1.WeatherBalloonSensors", ComponentSymbol.KIND);
        assertTrue(oCmp.isPresent());

        ComponentSymbol cmp = oCmp.get();
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

        Optional<PortSymbol> oPort = s.resolve("view.wbView.WCET1.WeatherBalloonSensors.controlSignalsIn", PortSymbol.KIND);
        assertTrue(oPort.isPresent());

        PortSymbol port = oPort.get();
        assertTrue(port.isIncoming());
        assertTrue(!port.isOutgoing());
    }

    @Test
    public void testSymtab_Connector(){
        Scope s = createSymTab("view.wbView.WCET2", resourcePath);

        Optional<ConnectorSymbol> oConn = s.resolve("view.wbView.WCET2.WeatherBalloonSensors.controlSignalsIndataSaveInternalOut", ConnectorSymbol.KIND);
        assertTrue(oConn.isPresent());

        ConnectorSymbol conn = oConn.get();
        assertTrue(conn.getSource().equals("controlSignalsIn"));
        assertTrue(conn.getTarget().equals("dataSaveInternalOut"));
        assertTrue(conn.getSourcePort().getName().equals("controlSignalsIn"));
        assertTrue(conn.getTargetPort().getName().equals("dataSaveInternalOut"));
    }

    @Test
    public void testSymtab_ComponentInstance() {
        Scope s = createSymTab("view.wbView.WCET2", resourcePath);

        Optional<ComponentInstanceSymbol> oCmpInst = s.resolve("view.wbView.WCET2.wbSens", ComponentInstanceSymbol.KIND);
        assertTrue(oCmpInst.isPresent());

        ComponentInstanceSymbol cmpInst = oCmpInst.get();
        ComponentSymbolReference type = cmpInst.getComponentType();
        assertTrue(type.getName().equals("WeatherBalloonSensors"));
    }
}
