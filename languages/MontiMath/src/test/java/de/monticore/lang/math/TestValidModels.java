/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.lang.math._cocos.MathCocos;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by Tobias PC on 30.12.2016.
 */
public class TestValidModels extends AbstractMathChecker {
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

    private static String MODEL_PATH_VALID = "src/test/resources/symtab/";

    @Test
    public void testValidModels() {
        String modelName = "ValidExamples.m";
        testModelNoErrors(MODEL_PATH_VALID + modelName);
    }
}
