package de.monticore.lang.monticar.borealis.runtime.language.options;

import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;

public interface OptionsRegistry {
    Options getOptions();
    void registerOption(Option option);
}
