package de.monticore.lang.monticar.generator.middleware.compile;

public abstract class WindowsCompilationGenerator extends CompilationGenerator {
    private String PATH_TEMPLATE = "IF NOT [%<new_exe>_HOME%] == [] (\n" +
            "   set PATH=\"%<new_exe>_HOME%;%PATH%\"\n" +
            ")";
    private String CHECK_EXE_TEMPLATE = "where <exe>\n" +
            "IF NOT %ERRORLEVEL% EQU 0 (\n" +
            "   echo \"Can not find <exe> in PATH! Aborting.\"\n" +
            "   exit /B 1\n" +
            ")";
    private String SOURCE_ENV_VARS_TEMPLATE = "call <env_file>";

    @Override
    public String getPathTemplate() {
        return PATH_TEMPLATE;
    }

    @Override
    public String getCheckExeTemplate() {
        return CHECK_EXE_TEMPLATE;
    }

    @Override
    public String getSourceEnvVarsTemplate() {
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
}
