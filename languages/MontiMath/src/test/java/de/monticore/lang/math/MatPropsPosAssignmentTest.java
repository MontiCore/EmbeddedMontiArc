/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.lang.math._cocos.MathCocos;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by Philipp Goerick on 26.09.2017.
 */

public class MatPropsPosAssignmentTest extends AbstractMathChecker {

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
        String modelName = "MatrixPropertiesPos.m";
        testModelNoErrors(MODEL_PATH_INVALID + modelName);
    }

}

