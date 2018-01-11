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
package de.monticore.lang.math.math;

import de.monticore.lang.math.math._cocos.MathCoCoChecker;
import de.monticore.lang.math.math._cocos.MathCocos;
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
