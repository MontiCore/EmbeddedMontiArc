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
package de.monticore.lang.monticar.emadl.cocos;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.emadl._cocos.DataPathCocos;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;


import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;

public class TaggingCoCoTest extends AbstractTaggingResolverTest {
    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
    }

    @Test
    public void testCoCosWithInvalidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.Alexnet", EMAComponentSymbol.KIND)
            .orElse(null);
        assertNotNull(symbol);

        checkValid(symbol, tagging);

        assertFalse(Log.getFindings().isEmpty());
    }

    @Test
    public void testCoCosWithValidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.CorrectTypeNet", EMAComponentSymbol.KIND)
            .orElse(null);
        assertNotNull(symbol);

        checkValid(symbol, tagging);

        assertTrue(Log.getFindings().isEmpty());
    }

    //@Test
    public void testCoCosForInstancesWithValidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources/");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.CorrectTypeInstance.net1", EMAComponentSymbol.KIND)
            .orElse(null);
        //assertNotNull(symbol);

        checkValid(symbol, tagging);

        assertTrue(Log.getFindings().isEmpty());
    }

    /**
     * Checks all cocos on the given node, and checks for absence of errors. Use this for checking
     * valid models.
     */
    protected static void checkValid(EMAComponentSymbol symbol, TaggingResolver tagging) {
        Log.getFindings().clear();
        DataPathCocos.check(symbol, tagging);
    }

    protected static void checkValid(EMAComponentInstanceSymbol instance, TaggingResolver tagging) {
        Log.getFindings().clear();
        DataPathCocos.check(instance, tagging);
    }
}
