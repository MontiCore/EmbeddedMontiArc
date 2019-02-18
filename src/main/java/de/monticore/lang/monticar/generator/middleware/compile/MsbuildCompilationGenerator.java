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
    public String getScriptTemplate() {
        return TemplateHelper.getCompilationMsbuildTemplate();
    }

    @Override
    public List<String> getAdditionalPathDirs() {
        return Arrays.asList("cmake","msbuild");
    }


    @Override
    protected String getFileName() {
        return "compileMsbuild.bat";
    }

    @Override
    protected List<String> getEnvironmentFiles() {
        List<String> res = new ArrayList<>();
        res.add("vcvars64.bat");
        if(useRos2()){
            res.add("%ROS_HOME%\\local_setup.bat");
        }
        return res;
    }

    @Override
    protected List<String> getExecutables() {
        return Arrays.asList("cmake","vcvars64.bat","msbuild");
    }
}
