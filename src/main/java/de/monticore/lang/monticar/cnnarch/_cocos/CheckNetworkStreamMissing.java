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
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.UnrollSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckNetworkStreamMissing extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        boolean hasNetworkStream = false;

        for (CompositeElementSymbol stream : architecture.getStreams()) {
            hasNetworkStream |= stream.isNetwork();
        }

        for (UnrollSymbol unroll : architecture.getUnrolls()) {
            hasNetworkStream |= unroll.isNetwork();
        }

        if (!hasNetworkStream) {
            Log.error("0" + ErrorCodes.MISSING_NETWORK_STREAM + " The architecture has no network stream. "
                    , architecture.getSourcePosition());
        }
    }

}
