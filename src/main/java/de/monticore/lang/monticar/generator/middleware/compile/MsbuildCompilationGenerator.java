package de.monticore.lang.monticar.generator.middleware.compile;

import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MsbuildCompilationGenerator extends WindowsCompilationGenerator {
    @Override
    public boolean supportsRos2() {
        return true;
    }

    @Override
    protected String getScriptTemplate() {
        return TemplateHelper.getCompilationMsbuildTemplate();
    }

    @Override
    protected List<String> getAdditionalPathDirs() {
        setAdditionalErrorMsg("vcvars64.bat", defaultErrorMsg("msbuild"));
        setAdditionalErrorMsg("ros2", defaultErrorMsg("ROS2"));
        return Arrays.asList("cmake","msbuild");
    }


    @Override
    protected String getFileName() {
        return "compileMsbuild.bat";
    }

    @Override
    protected List<String> getPostSourceExecutables() {
        List<String> res = new ArrayList<>();
        res.add("msbuild");
        if(useRos2()){
            res.add("ros2");
        }
        return res;
    }

    @Override
    protected List<String> getEnvironmentFiles() {
        List<String> res = new ArrayList<>();
        res.add("vcvars64.bat");
        if(useRos2()){
            res.add("%ROS2_HOME%\\local_setup.bat");
        }
        return res;
    }

    @Override
    protected List<String> getExecutables() {
        return Arrays.asList("cmake","vcvars64.bat");
    }
}
