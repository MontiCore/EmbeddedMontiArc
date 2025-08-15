/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator.preprocessing;

import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.ListIterator;
import java.util.Set;

/**
 *
 */
public class PreprocessingPortChecker {

    public PreprocessingPortChecker() { }

    static public void check(final PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        assert preprocessingComponentParameter != null;

        checkEqualNumberofInAndOutPorts(preprocessingComponentParameter);
        checkCorrectPortNames(preprocessingComponentParameter);
    }

    static private void checkEqualNumberofInAndOutPorts(PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        failIfConditionFails(equalNumberOfInAndOutPorts(preprocessingComponentParameter),
                "The number of in- and output ports of the " +
                        "preprocessing component is not equal");
    }

    static private boolean equalNumberOfInAndOutPorts(PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        return preprocessingComponentParameter.getInputNames().size()
                == preprocessingComponentParameter.getOutputNames().size();
    }

    static private void checkCorrectPortNames(PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        failIfConditionFails(correctPortNames(preprocessingComponentParameter),
                "The output ports are not correctly named with \"_out\" appendix");
    }

    static private boolean correctPortNames(PreprocessingComponentParameterAdapter preprocessingComponentParameter) {
        ListIterator<String> iterator = preprocessingComponentParameter.getInputNames().listIterator();
        Set<String> inputs = new HashSet<String>();
        while (iterator.hasNext()) {
            inputs.add(iterator.next() + "_out");
        }
        Set<String> outputs = new HashSet<String>(preprocessingComponentParameter.getOutputNames());
        return inputs.equals(outputs);
    }

    static private void failIfConditionFails(final boolean condition, final String message) {
        if (!condition) {
            fail(message);
        }
    }

    static private void fail(final String message) {
        Log.error(message);
        //System.exit(-1);
    }
}