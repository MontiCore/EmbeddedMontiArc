/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
public class ComponentInstanceSymbolLibraryFactory {
    public static final String ALL_TYPES_MODEL = "all-types";
    public static final String IO_TEST_MODEL = "io-test";
    public static final String CALCULATOR_MODEL = "calculator";
    public static final String NO_INPUT_MODEL = "no-input";
    public static final String NO_OUTPUT_MODEL = "no-output";
    public static final String NO_INPUT_NO_OUTPUT_MODEL = "no-input-no-output";
    public static final String DEEP_LEARNING_COMPONENT = "deep-learning-component";

    private static ComponentInstanceSymbolLibrary library;

    public ComponentInstanceSymbolLibraryFactory() {

    }

    public ComponentInstanceSymbolLibrary getComponentInstanceLibrary() {
        if (library == null) {
            library = new ComponentInstanceSymbolLibrary();
            loadLibrary();
        }
        return library;
    }

    private void loadLibrary() {
        checkNotNull(library);
        library.addInstanceToLibrary(
                ALL_TYPES_MODEL,
                "target/test-classes/all-types-model/",
                "types.allTypes");
        library.addInstanceToLibrary(
                IO_TEST_MODEL,
                "target/test-classes/io-test",
                "io.compTestIO"
        );
        library.addInstanceToLibrary(
                CALCULATOR_MODEL,
                "target/test-classes/calculator",
                "calculator.calculator"
        );
        library.addInstanceToLibrary(
                NO_INPUT_MODEL,
                "target/test-classes/noinput",
                "test.noInputComponent"
        );
        library.addInstanceToLibrary(
                NO_OUTPUT_MODEL,
                "target/test-classes/nooutput",
                "test.noOutputComponent"
        );
        library.addInstanceToLibrary(
                NO_INPUT_NO_OUTPUT_MODEL,
                "target/test-classes/noinputnooutput",
                "test.noInputNoOutputComponent"
        );
        library.addInstanceToLibrary(
                DEEP_LEARNING_COMPONENT,
                "target/test-classes/dl-model",
                "dl.deepLearningComponent"
        );
    }
}
