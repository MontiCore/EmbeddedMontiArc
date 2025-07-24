package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

public enum BooleanInputPort implements IInputPort{
    TRUE_INPUT(true),
    FALSE_INPUT(false);

    private final boolean inputValue;

    BooleanInputPort(boolean expectedValue) {
        this.inputValue = expectedValue;
    }

    public boolean getInputValue() {
        return inputValue;
    }
}
