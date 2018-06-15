/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.mathopt.OptimizationModelHelper;
import de.monticore.lang.mathopt._ast.ASTMathOptNode;
import de.monticore.lang.mathopt._parser.MathOptParser;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;

import java.util.Optional;

/**
 * Abstract base class to create coco tests. Provides methods to generate an AST node from string and assign symbols to it
 *
 * @author Christoph Richter
 */
public abstract class AbstractMathOptCocoTest {

    private MathOptParser parser = new MathOptParser();

    protected MathOptParser getParser() {
        return parser;
    }

    protected void initializeSymbol(ASTMathOptNode astNode) {
        Optional<MathOptSymbolTableCreator> stc = OptimizationModelHelper.getInstance().getSymbolTableCreator(astNode, Optional.empty());
    }

}
