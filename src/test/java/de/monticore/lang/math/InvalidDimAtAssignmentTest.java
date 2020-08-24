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
 * Created by Tobias PC on 30.12.2016.
 */
public class InvalidDimAtAssignmentTest extends AbstractMathChecker {
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
    public void testInvalidDimAtAssignment() {
        String modelName = "InvalidDimAtAssignment.m";
        String errorCode1 = "0xMATH02";

        Collection<Finding> expectedErrors = Arrays
                .asList(
                        Finding.error(errorCode1 + " Wrong Dimension at assignment"),
                        Finding.error(errorCode1 + " Wrong Dimension at assignment")
                );
        testModelForErrors(MODEL_PATH_INVALID + modelName, expectedErrors);
    }
}
