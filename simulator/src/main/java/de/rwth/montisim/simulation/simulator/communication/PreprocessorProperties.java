package de.rwth.montisim.simulation.simulator.communication;

/**
 * Abstract class that defines the parameters of every preprocessor
 * and a function that build a certain preprocessor.
 */
public abstract class PreprocessorProperties {
    public abstract Preprocessor build();
}
