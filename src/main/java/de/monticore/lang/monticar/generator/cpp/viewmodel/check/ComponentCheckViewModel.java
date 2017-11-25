package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.Collections;
import java.util.Map;

public final class ComponentCheckViewModel extends ViewModelBase {

    private Map<String, String> inputPortName2Value = Collections.emptyMap();
    private Map<String, IOutputPortCheck> outputPortName2Check = Collections.emptyMap();

    public Map<String, String> getInputPortName2Value() {
        return inputPortName2Value;
    }

    public void setInputPortName2Value(Map<String, String> inputPortName2Value) {
        this.inputPortName2Value = inputPortName2Value;
    }

    public Map<String, IOutputPortCheck> getOutputPortName2Check() {
        return outputPortName2Check;
    }

    public void setOutputPortName2Check(Map<String, IOutputPortCheck> outputPortName2Check) {
        this.outputPortName2Check = outputPortName2Check;
    }
}
