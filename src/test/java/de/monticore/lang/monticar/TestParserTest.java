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
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.test._ast.ASTA;
import de.monticore.lang.monticar.test._ast.ASTC;
import de.monticore.lang.monticar.test._ast.ASTD;
import de.monticore.lang.monticar.test._parser.TestParser;
import de.se_rwth.commons.logging.Log;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha
 */
public class TestParserTest {

    static boolean failQuick;

    @BeforeClass
    public static void startUp() {
        failQuick = Log.isFailQuickEnabled();
        Log.enableFailQuick(false);
    }

    @AfterClass
    public static void tearDown() {
        Log.enableFailQuick(failQuick);
    }

    @Before
    public void clear() {
        Log.getFindings().clear();
    }

    @Test
    public void testA() throws IOException {
        TestParser parser = new TestParser();
        ASTA ast = parser.parse_StringA("A.*;").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testC() throws IOException {
        TestParser parser = new TestParser();
        ASTC ast = parser.parse_StringC("C;").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testD() throws IOException {
        TestParser parser = new TestParser();
        ASTD ast = parser.parse_StringD("D;").orElse(null);
        assertNotNull(ast);
    }


    /*@Test
    public void testB() throws IOException {
        TestParser parser = new TestParser();
        ASTB ast = parser.parseString_B("B.*;").orElse(null);
        assertNotNull(ast);
    }*/

}

