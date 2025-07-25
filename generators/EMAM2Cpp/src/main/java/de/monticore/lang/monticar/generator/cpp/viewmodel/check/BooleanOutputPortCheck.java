/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.viewmodel.check;

public enum BooleanOutputPortCheck implements IOutputPortCheck {
    TRUE_EXPECTED(true),
    FALSE_EXPECTED(false);

    private final boolean expectedValue;

    BooleanOutputPortCheck(boolean expectedValue) {
        this.expectedValue = expectedValue;
    }

    public boolean getExpectedValue() {
        return expectedValue;
    }
}
