/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.lang.math._cocos.MathCocos;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collection;

/**
 * Created by Philipp Goerick on 26.09.2017.
 */

public class MatPropsNegAssignmentTest extends AbstractMathChecker {

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

    private static String MODEL_PATH_INVALID = "src/test/resources/matrix/";

    @Test
    public void assignmentTest(){
        String modelName = "MatrixPropertiesNeg.m";
        Collection<Finding> expectedErrors = Arrays
                .asList(
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties"),
                        Finding.error("Matrix does not fullfill given properties")
                        //Finding.error("Matrix does not fullfill given properties")
                );

        testModelForErrors(MODEL_PATH_INVALID + modelName, expectedErrors);
    }

}
