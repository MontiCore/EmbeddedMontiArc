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
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.tools.Slist;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.prettyprint.IndentPrinter;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Created by ernst on 13.08.2016.
 */
public class SListTest extends AbstractSymtabTest {

    @Test
    public void testSList() {
        Slist.resetNonVirtualBlockSize();
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "fas.advancedLibrary.rSFlipFlop", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        IndentPrinter ip = new IndentPrinter();
        ip.println("---- Sorted list for 'rSFlipFlop' [6 nonvirtual block(s), directfeed=0]");
        ip.indent();
        ip.println("0:0    'fas.advancedLibrary.rSFlipFlop.oneS' (Constant, tid=PRM)");
        ip.println("0:1    'fas.advancedLibrary.rSFlipFlop.zeroR' (Constant, tid=PRM)");
        ip.println("0:2    'fas.advancedLibrary.rSFlipFlop.memory_Q' (Memory, tid=0)");
        ip.println("0:3    'fas.advancedLibrary.rSFlipFlop.switch_S' (SwitchB, tid=0)");
        ip.println("0:4    'fas.advancedLibrary.rSFlipFlop.switch_R' (SwitchB, tid=0)");
        ip.println("0:5    'fas.advancedLibrary.rSFlipFlop.logOp_N' (Not, tid=0)");
        ip.unindent();
        assertEquals(ip.getContent(), Slist.execute(symTab, inst));
        Log.info(Slist.execute(symTab, inst), "SLIST");
        Log.info("", "testSList Done");
    }

    @Test
    public void testSlistDetection() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "detection.objectDetector", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (ExpandedComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }

    @Test
    public void testSlistMatrixModifier() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "paper.matrixModifier", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (ExpandedComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }

    @Test
    public void testSlistMathUnit() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (ExpandedComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }
}
