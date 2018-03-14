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
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.IOException;

public class AllCoCoTest extends AbstractCoCoTest{

    public AllCoCoTest() {
        Log.enableFailQuick(false);
    }

    @Test
    public void testValidCoCos() throws IOException {
        checkValid("valid_tests","SimpleConfig1");
        checkValid("valid_tests","SimpleConfig2");
        checkValid("valid_tests","FullConfig");
        checkValid("valid_tests","FullConfig2");
    }

    @Test
    public void testInvalidCoCos() throws IOException {
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckEntryRepetition()),
                "invalid_cocos_tests", "EntryRepetition",
                new ExpectedErrorInfo(1, ErrorCodes.ENTRY_REPETITION_CODE));
        checkInvalid(new CNNTrainCoCoChecker().addCoCo(new CheckInteger()),
                "invalid_cocos_tests", "IntegerTest",
                new ExpectedErrorInfo(1, ErrorCodes.NOT_INTEGER_CODE));
    }
}
