package de.monticore.lang.monticar.generator.middleware.compile;

import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MingwCompilationGenerator extends WindowsCompilationGenerator {

    @Override
    public boolean supportsRos2() {
        return false;
    }

    @Override
    protected String getScriptTemplate() {
        return TemplateHelper.getCompilationMingwTemplate();
    }

    @Override
    protected List<String> getAdditionalPathDirs() {
        return Arrays.asList("cmake","make","g++");
    }

    @Override
    protected String getFileName() {
        return "compileMingw.bat";
    }

    @Override
    protected List<String> getPostSourceExecutables() {
        return new ArrayList<>();
    }

    @Override
    protected List<String> getEnvironmentFiles() {
        return new ArrayList<>();
    }

    @Override
    protected List<String> getExecutables() {
        return Arrays.asList("cmake","make","g++");
    }
}
