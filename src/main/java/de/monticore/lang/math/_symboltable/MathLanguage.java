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
package de.monticore.lang.math._symboltable;

import de.monticore.lang.math.LogConfig;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

/**
 * @author math-group
 */

public class MathLanguage extends MathLanguageTOP{
    /** known file ending from matlab "*.m" */
    public static final String FILE_ENDING = "m";

    public MathLanguage() {
        super("Math Language", FILE_ENDING);
        LogConfig.init();
    }

    @Override
    protected MathModelLoader provideModelLoader() {
        return new MathModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        //addResolver(CommonResolvingFilter.create(AssignmentSymbol.KIND));
        //addResolver(CommonResolvingFilter.create(MathVariableDeclarationSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MathExpressionSymbol.KIND));
        addResolver(CommonResolvingFilter.create(MathStatementsSymbol.KIND));

        setModelNameCalculator(new MathModelNameCalculator());
    }
}
