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
package de.monticore.lang.monticar.cnntrain.cocos;

import de.monticore.lang.monticar.cnntrain._cocos.*;
import org.junit.Test;

import java.io.IOException;

public class AllCoCoTest extends AbstractCoCoTest{
    String baseDir="src/test/resources";
    @Test
    public void testCoCosSimulator() throws IOException {
        checkValid("","SimpleConfig1");
        checkValid("","SimpleConfig2");
        checkValid("","DataReferenceTest");
        checkValid("","FullConfig");
        checkValid("","FullConfig2");

        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckEntryRepetition()),
                getAstNode("", "DuplicatedParameter"),
                new ExpectedErrorInfo(1, "xC8853"));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckInteger()),
                getAstNode("", "IntegerTest"),
                new ExpectedErrorInfo(1, "xC8851"));
        /*checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckValidPath()),
                getAstNode("", "InvalidPath"),
                new ExpectedErrorInfo(1, "xC8856"));*/

    }
}
