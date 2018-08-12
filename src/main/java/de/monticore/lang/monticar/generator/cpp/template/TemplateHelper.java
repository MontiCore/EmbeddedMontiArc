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
package de.monticore.lang.monticar.generator.cpp.template;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.BooleanOutputPortCheck;
import de.monticore.lang.monticar.generator.cpp.viewmodel.check.RangeOutputPortCheck;
import de.se_rwth.commons.logging.Log;

import java.util.HashMap;
import java.util.Map;

public final class TemplateHelper {

    public static final TemplateHelper INSTANCE = new TemplateHelper();

    private TemplateHelper() {
    }

    public boolean isBooleanOutputPortCheck(Object check) {
        return check instanceof BooleanOutputPortCheck;
    }

    public boolean isRangeOutputPortCheck(Object check) {
        return check instanceof RangeOutputPortCheck;
    }

    public boolean isTrueExpectedCheck(Object check) {
        return BooleanOutputPortCheck.TRUE_EXPECTED.equals(check);
    }

    public boolean isFalseExpectedCheck(Object check) {
        return BooleanOutputPortCheck.FALSE_EXPECTED.equals(check);
    }

    public static Map<String, Object> getDataForTemplate(ViewModelBase viewModel) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("viewModel", Log.errorIfNull(viewModel));
        data.put("helper", INSTANCE);
        return data;
    }
}
