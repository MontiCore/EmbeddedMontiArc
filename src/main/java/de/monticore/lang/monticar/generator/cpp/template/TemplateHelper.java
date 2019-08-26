/* (c) https://github.com/MontiCore/monticore */
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

    public boolean isRangeOutputPortCheckMatrix(Object check) {
        if (isRangeOutputPortCheck(check)) {
            return ((RangeOutputPortCheck) check).isMatrix();
        }
        return false;
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
