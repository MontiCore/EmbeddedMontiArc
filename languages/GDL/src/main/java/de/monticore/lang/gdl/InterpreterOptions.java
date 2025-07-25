package de.monticore.lang.gdl;

public class InterpreterOptions {
    
    private boolean
        manualRandom = false,
        debugMode = false,
        showTimes = false,
        withTypes = false;

    public InterpreterOptions manualRandom(boolean manualRandom) {
        this.manualRandom = manualRandom;
        return this;
    }

    public InterpreterOptions debugMode(boolean debugMode) {
        this.debugMode = debugMode;
        return this;
    }

    public InterpreterOptions showTimes(boolean showTimes) {
        this.showTimes = showTimes;
        return this;
    }

    public InterpreterOptions withTypes(boolean withTypes) {
        this.withTypes = withTypes;
        return this;
    }

    public boolean isManualRandom() {
        return manualRandom;
    }
    
    public boolean isDebugMode() {
        return debugMode;
    }

    public boolean isShowTimes() {
        return showTimes;
    }

    public boolean isWithTypes() {
        return withTypes;
    }

}
