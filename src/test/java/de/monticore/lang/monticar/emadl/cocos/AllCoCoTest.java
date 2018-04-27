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

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;

public class AllCoCoTest extends AbstractCoCoTest {
    String baseDir="src/test/resources";

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testCoCosSimulator() throws IOException {
        checkValid("", "mnist.Main");
        checkValid("", "Alexnet");
        checkValid("", "VGG16");
        checkValid("", "ThreeInputCNN_M14");
        checkValid("", "MultipleOutputs");
        checkValid("", "ResNet34");
        checkValid("", "ResNet152");
        checkValid("", "ResNeXt50");

        checkValid("", "simulator.MainController");

        checkValid("", "Add");
        checkValid("", "simulator.SteeringAngleCalculator");

        /*checkInvalid(new EMADLCoCoChecker().addCoCo(new CheckArchitectureIO()),
                new EMADLCoCoChecker(),
                "", "InvalidIOAndArgs",
                new ExpectedErrorInfo(2, ErrorCodes.MISSING_IO_CODE, ErrorCodes.UNKNOWN_IO_CODE));
*/
    }

}