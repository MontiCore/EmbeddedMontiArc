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

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;

public class AllCoCoTest extends AbstractCoCoTest {
    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testCoCosValid(){
        checkValid("models", "cifar10.Cifar10Classifier");
        checkValid("models", "Alexnet");
        checkValid("models", "VGG16");
        checkValid("models", "ThreeInputCNN_M14");
        checkValid("models", "MultipleOutputs");
        checkValid("models", "ResNet34");
        checkValid("models", "ResNet152");
        checkValid("models", "ResNeXt50");
        checkValid("models", "RNNsearch");

        checkValid("models", "simulator.MainController");

        checkValid("models", "Add");
        checkValid("models", "simulator.SteeringAngleCalculator");

    }

    @Test
    public void testInvalidLayerInput(){
        checkInvalid("models",
                "InvalidLayerInput",
                new ExpectedErrorInfo(1, ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE));
    }

    @Test
    public void testMathOpt(){
        checkValid("models", "MinimizePortsTest");
    }

}
