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
package de.monticore.lang.mathopt._symboltable;

import de.monticore.lang.math.LogConfig;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathModelNameCalculator;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.Collection;
import java.util.Optional;

/**
 * @author Christoph Richter
 */
public class MathOptLanguage extends MathOptLanguageTOP {

    private MathLanguage parentLanguage;

    public MathOptLanguage() {
        super("Math Optimization Language", MathLanguage.FILE_ENDING);
        parentLanguage = new MathLanguage();
        LogConfig.init();
    }

    @Override
    protected MathOptModelLoader provideModelLoader() {
        return new MathOptModelLoader(this);
    }

    @Override
    public Optional<MathOptSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope mutableScope) {
        return Optional.of(new MathOptSymbolTableCreator(resolvingConfiguration, mutableScope));
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        addResolvingFilter(CommonResolvingFilter.create(MathExpressionSymbol.KIND));
        addResolvingFilter(CommonResolvingFilter.create(MathStatementsSymbol.KIND));

        setModelNameCalculator(new MathModelNameCalculator());
    }
}
