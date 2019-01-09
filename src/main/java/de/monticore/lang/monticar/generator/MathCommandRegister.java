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
package de.monticore.lang.monticar.generator;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public abstract class MathCommandRegister {
    public List<MathCommand> mathCommands = new ArrayList<>();

    public MathCommandRegister() {
        init();
    }

    public void registerMathCommand(MathCommand mathCommand) {
        mathCommands.add(mathCommand);
    }

    public MathCommand getMathCommand(String functionName) {
        for (MathCommand mathCommand : mathCommands) {
            if (mathCommand.getMathCommandName().equals(functionName))
                return mathCommand;
        }
        return null;
    }

    public boolean isMathCommand(String functionName) {
        boolean isMathCommand = false;
        if (getMathCommand(functionName) != null) {
            isMathCommand = true;
        } else {
            isMathCommand = isTargetLanguageCommand(functionName);
        }
        return isMathCommand;
    }

    private boolean isTargetLanguageCommand(String command) {
        for (MathCommand mathCommand : mathCommands)
            if (mathCommand.isTargetLanguageCommand(command))
                return true;
        return false;
    }

    protected abstract void init();

}
