package de.monticore.lang.monticar.generator.roscpp.util;

/**
 * Instruction are used inside of Methods to add a symbolic representation of behaviour added by the Math
 * Language. They can also be used to add different Instructions to methods which are not related to
 * the Math language, like port connecting for setting inputs.
 *
 * @author Sascha Schneiders
 */
public interface Instruction {
    String getTargetLanguageInstruction();

    boolean isConnectInstruction();

    default boolean isTargetCodeInstruction() {
        return false;
    }

    default boolean isExecuteInstruction() {
        return false;
    }
}
