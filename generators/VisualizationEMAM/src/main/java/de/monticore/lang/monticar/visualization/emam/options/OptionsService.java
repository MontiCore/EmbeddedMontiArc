/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.options;

import org.apache.commons.cli.ParseException;

import java.io.File;
import java.nio.file.Path;

public interface OptionsService {
    void printHelp();
    String getOptionAsString(String option);
    Path getOptionAsPath(String option);
    File getOptionAsFile(String option);
}
