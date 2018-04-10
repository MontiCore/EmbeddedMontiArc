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
package de.monticore.lang.monticar.cnntrain._symboltable;

import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class CNNTrainLanguage extends CNNTrainLanguageTOP {

    public static final String FILE_ENDING = "cnnt";

    public CNNTrainLanguage() {
        super("CNNTrain Language", FILE_ENDING);
    }

    @Override
    protected CNNTrainModelLoader provideModelLoader() {
        return new CNNTrainModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        addResolvingFilter(new CNNTrainCompilationUnitResolvingFilter());
        addResolvingFilter(new CommonResolvingFilter<Symbol>(ConfigurationSymbol.KIND));
        addResolvingFilter(new CommonResolvingFilter<Symbol>(OptimizerSymbol.KIND));
        addResolvingFilter(new CommonResolvingFilter<Symbol>(OptimizerParamSymbol.KIND));
        setModelNameCalculator(new CNNTrainModelNameCalculator());
    }

}
