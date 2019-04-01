package de.monticore.lang.monticar.generator.middleware.compile;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;

import java.util.ArrayList;
import java.util.List;

public abstract class WindowsCompilationGenerator extends CompilationGenerator {
    private String PATH_TEMPLATE = "IF NOT [%<new_exe>_HOME%] == [] (\n" +
            "\tset PATH=\"%<new_exe>_HOME%;%PATH%\"\n" +
            ")";
    private String CHECK_EXE_TEMPLATE = "where <exe>\n" +
            "IF NOT %ERRORLEVEL% EQU 0 (\n" +
            "\techo \"Can not find <exe> in PATH! Aborting.\"\n" +
            "<additional_error>" +
            "\texit /B 1\n" +
            ")";
    private String SOURCE_ENV_VARS_TEMPLATE = "call <env_file>";

    @Override
    protected String getPathTemplate() {
        return PATH_TEMPLATE;
    }

    @Override
    protected String getCheckExeTemplate() {
        return CHECK_EXE_TEMPLATE;
    }

    @Override
    protected String getSourceEnvVarsTemplate() {
        return SOURCE_ENV_VARS_TEMPLATE;
    }

    @Override
    protected String getNewlineDelimiter() {
        return "\r\n";
    }

    @Override
    public boolean supportsRos() {
        return false;
    }

    private String fillSubstTemplate(String compileScript) {
        return TemplateHelper.getSubstTemplate()
                .replace("<compile_script>", compileScript);
    }

    @Override
    public List<FileContent> getCompilationScripts() {
        ArrayList<FileContent> fileContents = new ArrayList<>();
        fileContents.addAll(super.getCompilationScripts());

        FileContent substScript = new FileContent();
        substScript.setFileName("subst" + getFileName().substring(0,1).toUpperCase() + getFileName().substring(1));
        substScript.setFileContent(fillSubstTemplate(getFileName()));
        fileContents.add(substScript);

        return fileContents;
    }
}
