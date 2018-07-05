package de.monticore.lang.monticar.visualization.emam.options;

import org.apache.commons.cli.ParseException;

import java.nio.file.Path;

public interface OptionsService {
    void printHelp();
    Object getOption(String option) throws ParseException;
    String getOptionAsString(String option) throws ParseException;
    Path getOptionAsPath(String option) throws ParseException;
}
