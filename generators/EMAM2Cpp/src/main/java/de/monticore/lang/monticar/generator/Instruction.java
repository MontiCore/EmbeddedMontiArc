/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

/**
 * Instruction are used inside of Methods to add a symbolic representation of behaviour added by the Math
 * Language. They can also be used to add different Instructions to methods which are not related to
 * the Math language, like port connecting for setting inputs.
 *
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
