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
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import static junit.framework.TestCase.assertTrue;

public class IntegrationGluonTest extends IntegrationTest {
    public IntegrationGluonTest() {
        super("GLUON", "39253EC049D4A4E5FA0536AD34874B9D#1DBAEE1B1BD83FB7CB5F70AE91B29638#C4C23549E737A759721D6694C75D9771#5AF0CE68E408E8C1F000E49D72AC214A");
    }

    @Test
    public void testMultipleInputs() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/", "-r", "MultipleInputs", "-b", "GLUON"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().isEmpty());
    }
}
