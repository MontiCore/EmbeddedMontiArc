package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.monticar.generator.ExecuteInstruction;

/**
 * @author Sascha Schneiders
 */
public class ThreadingOptimizer {

    public static void resetID() {
        ExecuteInstruction.threadCounter = 0;
    }

}
