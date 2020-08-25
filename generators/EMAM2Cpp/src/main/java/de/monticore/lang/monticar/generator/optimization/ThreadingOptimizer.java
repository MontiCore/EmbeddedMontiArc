/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.monticar.generator.ExecuteInstruction;

/**
 */
public class ThreadingOptimizer {

    public static void resetID() {
        ExecuteInstruction.threadCounter = 0;
    }

}
