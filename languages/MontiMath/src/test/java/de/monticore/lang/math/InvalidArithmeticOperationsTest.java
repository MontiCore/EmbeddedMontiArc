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
public class InvalidArithmeticOperationsTest extends AbstractMathChecker {
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
    public void testInvalidArithmeticOperations() {
        String modelName = "InvalidArithmeticOperations.m";
        String errorCode1 = "0xMATH04";
        String errorCode2 = "0xMATH06";
        String errorCode3 = "0xMATH11";

        Collection<Finding> expectedErrors = Arrays
                .asList(
                        Finding.error(errorCode1 + " Matrix Addition with different Dimensions\nMatrix1: "+1+" x "+ 2+ "\nMatrix2: "+1+" x "+ 3),
                        Finding.error(errorCode2 + " Matrix Multiplication with wrong Dimensions\nMatrix1: "+3+" x "+ 2+ "\nMatrix2: "+3+" x "+ 2),
                        Finding.error(errorCode3 + " Matrix Power with different Column and Row Dimensions")
                );
        testModelForErrors(MODEL_PATH_INVALID + modelName, expectedErrors);
    }
}
