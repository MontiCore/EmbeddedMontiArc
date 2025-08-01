/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.lang.math._cocos.MathCocos;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collection;

/**
 * Created by math-group on 12.01.2017.
 */

public class InvalidRangeTest extends AbstractMathChecker {
    @Override
    protected MathCoCoChecker getChecker() {
        return MathCocos.createChecker();
    }
    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() {
        Log.getFindings().clear();
    }

    private static String MODEL_PATH_INVALID = "src/test/resources/symtab/";
    @Ignore
    @Test
    public void testInvalidRange() {

        String modelName = "InvalidRange.m";
        String errorCode1 = "0xMATH22";
        String errorCode2 = "0xMATH23";
        String errorCode3 = "0xMATH24";
        String errorCode4 = "0xMATH25";

        Collection<Finding> expectedErrors = Arrays
                .asList(
                        Finding.error(errorCode1 + " Wrong Range at Assignment"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode2 + " Range does not fit"),
                        Finding.error(errorCode3 + " Matrix Range does not fit"),
                        Finding.error(errorCode3 + " Matrix Range does not fit"),
                        Finding.error(errorCode3 + " Matrix Range does not fit"),
                        Finding.error(errorCode3 + " Matrix Range does not fit")
                );
        testModelForErrors(MODEL_PATH_INVALID + modelName, expectedErrors);
    }

}

