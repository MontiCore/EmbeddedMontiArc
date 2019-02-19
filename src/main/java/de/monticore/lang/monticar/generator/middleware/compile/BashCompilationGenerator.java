package de.monticore.lang.monticar.generator.middleware.compile;


import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BashCompilationGenerator extends CompilationGenerator {
    private String PATH_TEMPLATE = "if [ -n \"$<new_exe>_HOME\" ]\n" +
            "then\n"+
            "\texport PATH=\"$<new_exe>_HOME:$PATH\"\n" +
            "fi";

    private String CHECK_EXE_TEMPLATE = "if [[ `command -v <exe>` ]]\n" +
            "then\n" +
            "\techo \"Found <exe>\"\n" +
            "else\n" +
            "\techo \"Can not find <exe> in PATH! Aborting.\"\n" +
            "<additional_error>" +
            "\texit 1\n" +
            "fi";

    private String SOURCE_ENV_VARS_TEMPLATE = "source <env_file>";


    @Override
    public boolean supportsRos() {
        return true;
    }

    @Override
    public boolean supportsRos2() {
        return true;
    }

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
    protected String getScriptTemplate() {
        return TemplateHelper.getCompilationBashTemplate().replace("\r\n", "\n");
    }

    @Override
    protected List<String> getAdditionalPathDirs() {
        setAdditionalErrorMsg("roscore", defaultErrorMsg("ROS"));
        setAdditionalErrorMsg("ros2", defaultErrorMsg("ROS2"));
        return Arrays.asList("cmake","make");
    }

    @Override
    protected String getFileName() {
        return "compile.sh";
    }

    @Override
    protected String getNewlineDelimiter() {
        return "\n";
    }

    @Override
    protected List<String> getPostSourceExecutables() {
        ArrayList<String> res = new ArrayList<>();
        if(useRos()){
            res.add("roscore");
        }
        if(useRos2()){
            res.add("ros2");
        }
        return res;
    }

    @Override
    protected List<String> getEnvironmentFiles() {
        List<String> res = new ArrayList<>();
        if(useRos()){
            res.add("\"$ROS_HOME\"/setup.bash");
        }
        if(useRos2()){
            res.add("\"$ROS2_HOME\"/setup.bash");
        }
        return res;
    }

    @Override
    protected List<String> getExecutables() {
        return Arrays.asList("cmake","make");
    }

}
