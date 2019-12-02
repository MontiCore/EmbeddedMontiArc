/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
        checkValid("models", "Show_attend_tell");

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
