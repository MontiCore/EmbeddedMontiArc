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
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class DataPathConfigParserTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testDataPathConfigParserValidComponent() {
        DataPathConfigParser parser = new DataPathConfigParser("src/test/resources/architectures/data_paths.txt");

        String data_path = parser.getDataPath("ComponentName");
        assertTrue("Wrong data path returned", data_path.equals("/path/to/training/data"));
    }

    @Test
    public void testDataPathConfigParserInvalidComponent() {
        DataPathConfigParser parser = new DataPathConfigParser("src/test/resources/architectures/data_paths.txt");

        String data_path = parser.getDataPath("NotExistingComponent");
        assertTrue("For not listed components, null should be returned", data_path == null);
        assertTrue(Log.getFindings().size() == 1);
    }

    @Test
    public void testDataPathConfigParserInvalidPath() {
        DataPathConfigParser parser = new DataPathConfigParser("invalid/path/data_paths.txt");

        assertTrue(Log.getFindings().size() == 1);
    }
}
