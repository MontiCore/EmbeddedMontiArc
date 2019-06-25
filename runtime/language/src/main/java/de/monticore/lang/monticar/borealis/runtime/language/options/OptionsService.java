package de.monticore.lang.monticar.borealis.runtime.language.options;

import java.io.File;
import java.nio.file.Path;

public interface OptionsService {
    boolean hasOption(String option);

    String getOptionAsString(String option);
    String getOptionAsString(String option, String defaultValue);

    Path getOptionAsPath(String option);
    Path getOptionAsPath(String option, String defaultPath);

    File getOptionAsFile(String option);
    File getOptionAsFile(String option, String defaultFile);

    int getOptionAsInteger(String option);
    int getOptionAsInteger(String option, String defaultValue);
}
