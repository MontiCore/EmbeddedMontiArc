package de.monticore.lang.monticar.generator.middleware.compile;

import de.monticore.lang.monticar.generator.FileContent;

import java.util.*;
import java.util.stream.Collectors;

public abstract class CompilationGenerator {
    private boolean useRos = false;
    private boolean useRos2 = false;
    private Map<String, String> additionalErrorMsg = new HashMap<>();

    public abstract boolean supportsRos();

    public abstract boolean supportsRos2();

    public boolean isValid() {
        return (supportsRos() || !useRos()) && (supportsRos2() || !useRos2());
    }

    protected void setAdditionalErrorMsg(String executable, String errorMsg) {
        additionalErrorMsg.put(executable, errorMsg);
    }

    protected String getAdditionalErrorMsg(String executable) {
        String s = additionalErrorMsg.getOrDefault(executable, defaultErrorMsg(executable));
        return "\techo \"" + s + "\"" + getNewlineDelimiter();
    }

    protected abstract String getPathTemplate();

    protected abstract String getCheckExeTemplate();

    protected abstract String getSourceEnvVarsTemplate();

    protected abstract String getScriptTemplate();

    protected abstract List<String> getAdditionalPathDirs();

    protected abstract String getFileName();

    protected abstract String getNewlineDelimiter();

    public boolean useRos() {
        return useRos;
    }

    public void setUseRos(boolean useRos) {
        this.useRos = useRos;
    }

    public boolean useRos2() {
        return useRos2;
    }

    public void setUseRos2(boolean useRos2) {
        this.useRos2 = useRos2;
    }

    protected String fillPathTemplate(String newExe) {
        return getPathTemplate().replace("<new_exe>", newExe);
    }

    protected String fillCheckExeTemplate(String exe) {
        return getCheckExeTemplate()
                .replace("<exe>", exe)
                .replace("<additional_error>", getAdditionalErrorMsg(exe));
    }

    protected String fillSourceEnvVarsTemplate(String envFile) {
        return getSourceEnvVarsTemplate().replace("<env_file>", envFile);
    }

    protected String fillScriptTemplate(String additional_executables, String executable_checks, String additional_env, String post_executable_checks) {
        return getScriptTemplate()
                .replace("<additional_executables>", additional_executables)
                .replace("<executable_checks>", executable_checks)
                .replace("<additional_env>", additional_env)
                .replace("<post_executable_checks>", post_executable_checks);
    }

    public List<FileContent> getCompilationScripts() {
        FileContent res = new FileContent();
        String additional_executables = getAdditionalPathDirs().stream()
                .map(this::fillPathTemplate)
                .collect(Collectors.joining(getNewlineDelimiter()));

        String executable_checks = getExecutables().stream()
                .map(this::fillCheckExeTemplate)
                .collect(Collectors.joining(getNewlineDelimiter()));

        String additional_env = getEnvironmentFiles().stream()
                .map(this::fillSourceEnvVarsTemplate)
                .collect(Collectors.joining(getNewlineDelimiter()));

        String post_executable_checks = getPostSourceExecutables().stream()
                .map(this::fillCheckExeTemplate)
                .collect(Collectors.joining(getNewlineDelimiter()));

        res.setFileName(getFileName());
        res.setFileContent(fillScriptTemplate(additional_executables, executable_checks, additional_env, post_executable_checks));

        return Arrays.asList(res);
    }

    protected abstract List<String> getPostSourceExecutables();

    protected abstract List<String> getEnvironmentFiles();

    protected abstract List<String> getExecutables();

    public static List<CompilationGenerator> getInstanceOfAllGenerators() {
        List<CompilationGenerator> res = new ArrayList<>();
        res.add(new BashCompilationGenerator());
        res.add(new MingwCompilationGenerator());
        res.add(new MsbuildCompilationGenerator());
        return res;
    }

    protected String defaultErrorMsg(String executable) {
        return "Try setting the environment variable " + executable + "_HOME to the base of your installation or adding it to your PATH!";
    }
}
