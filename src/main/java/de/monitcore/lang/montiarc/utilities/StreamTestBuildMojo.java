package de.monitcore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.tools.AllTemplates;
import de.monitcore.lang.montiarc.utilities.tools.SearchFiles;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateExceptionHandler;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;

import java.io.*;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StreamTestBuildMojo extends StreamTestMojoBase {

    private static final Template TEMPLATE_BUILDFILE_UNIX;
    private static final Template TEMPLATE_BUILDFILE_WINDOWS;
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/");
        try {
            TEMPLATE_BUILDFILE_UNIX = conf.getTemplate("build/linux_build.ftl");
            TEMPLATE_BUILDFILE_WINDOWS = conf.getTemplate("build/windows_build.ftl");
        } catch (IOException e) {
            String msg = "could not load cmake templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    @Override
    protected void preExecution() throws MojoExecutionException, MojoFailureException {
        super.preExecution();

        Log.info("StreamTestBuildMojo", "StreamTestBuildMojo");

        if(!hasMojoRunned("StreamTestGeneratorMojo")){
            logInfo("StreamTestGeneratorMojo was not executed -> Run StreamTestGeneratorMojo");
            StreamTestGeneratorMojo stgm = new StreamTestGeneratorMojo();
            this.copyPropertiesAndParametersTo(stgm);
            stgm.execute();
        }

        Log.info("StreamTestBuildMojo", "StreamTestBuildMojo");
    }

    @Override
    protected void mainExecution() throws MojoExecutionException, MojoFailureException {
        createBuildFiles();

        if (SystemUtils.IS_OS_WINDOWS) {
            logInfo("Building with cmake for: \""+this.getGeneratorString(this.generator)+"\" and options: \""+this.getGeneratorBuildOptions(this.generator)+"\"");
        }else{
            logInfo("Building with cmake for: make");
        }


        buildStreamTests();

    }

    @Override
    protected String MojoName() {
        return "StreamTestBuildMojo";
    }


    //<editor-fold desc="Buildfile creation">
    protected void createBuildFiles() throws MojoExecutionException {
        logInfo("Create Build file(s)");
        Map root = new HashMap();
        root.put("genNONE", this.generator == GeneratorEnum.NONE);
        root.put("generator", this.getGeneratorString(this.generator));
        root.put("buildoptions", this.getGeneratorBuildOptions(this.generator));



        FileWriter fw = null;
        try {
            logInfo(" - build.sh");
            File f = Paths.get(this.getPathTmpOutCPP(), "build.sh").toFile();
            f.getParentFile().mkdirs();
            fw = new FileWriter(f);
            fw.write(AllTemplates.generate(TEMPLATE_BUILDFILE_UNIX, root));
            fw.flush();
            fw.close();

            logInfo(" - build.bat");
            f = Paths.get(this.getPathTmpOutCPP(), "build.bat").toFile();
            f.getParentFile().mkdirs();
            fw = new FileWriter(f);
            fw.write(AllTemplates.generate(TEMPLATE_BUILDFILE_WINDOWS, root));
            fw.flush();
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Can't create build file!"+e.getMessage());
        }

    }

    protected String getGeneratorString(GeneratorEnum gen){
        switch (gen){
            case MinGW:
                return "MinGW Makefiles";
            case VS2017:
            case VisualStudio2017:
                return "Visual Studio 15 2017 Win64";
            default:
                return "";
        }
    }

    protected String getGeneratorBuildOptions(GeneratorEnum gen){
        switch (gen){
            case VS2017:
            case VisualStudio2017:
                return "--config Debug";
            default:
                return "";
        }
    }
    //</editor-fold>

    //<editor-fold desc="Building">

    protected void buildStreamTests() throws MojoExecutionException, MojoFailureException {

        List<ComponentSymbol> toBuild = getToTestComponentSymbols(false);


        logInfo("Build StreamTest executables:");

        boolean allCompiled = true;

        for (ComponentSymbol cs : toBuild) {
            logInfo(" - "+cs.getFullName());
            allCompiled = allCompiled && buildName(cs.getFullName());
        }

        if(!allCompiled){
            throw new MojoFailureException("Not all streamtest were builded");
        }
    }

    protected boolean buildName(String name) throws MojoExecutionException {
        long startTime = System.nanoTime();
        ProcessBuilder processBuilder;
        if (SystemUtils.IS_OS_WINDOWS) {
            processBuilder = new ProcessBuilder(Paths.get(this.getPathTmpOutCPP(), name, "../build.bat").toAbsolutePath().toString());
        }else{
            processBuilder=new ProcessBuilder("/bin/bash", "../build.sh");
        }
        processBuilder.directory(Paths.get(this.getPathTmpOutCPP(), name).toFile());
        try {
            Process process = processBuilder.start();
            List<String> allLines = new ArrayList<>();
            if (showBuildAndRunOutput) {
                new BufferedReader(new InputStreamReader(process.getInputStream())).lines().
                        forEach((String s) -> this.logInfo("   [build-info] "+s));
                new BufferedReader(new InputStreamReader(process.getErrorStream())).lines().
                        forEach((String s) -> this.logError("   [build-error] "+s));
            } else {
                BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream()));
                String line;
                while (((line = in.readLine()) != null)) {
                    allLines.add(line);
                }
            }
            process.waitFor();

            if(!Paths.get(this.getPathTmpOutCPP(), name, execFileName(this.generator)).toFile().exists()){
                for(String l : allLines){
                    logError("    # Build error out: "+l);
                }
                return false;
            }else{

                long stopTime = System.nanoTime();

                logInfo("   -> Success in "+getReadableTime(stopTime-startTime));
            }

        }catch (Exception ex){
            ex.printStackTrace();
            throw new MojoExecutionException(ex.getMessage());
        }
        return true;
    }

    protected String execFileName(GeneratorEnum generator){
        if (SystemUtils.IS_OS_WINDOWS) {
            switch (generator) {
                case VS2017:
                case VisualStudio2017:
                    return "build/Debug/StreamTests.exe";
                default:
                    return "build/StreamTests.exe";
            }
        }else{
            return "build/StreamTests";
        }
    }

    private static String getReadableTime(Long nanos){

        long tempSec = nanos/(1000*1000*1000);
        long ms = (nanos/(1000*1000))%1000;
        long sec = tempSec % 60;
        long min = (tempSec /60) % 60;
        long hour = (tempSec /(60*60));
        return String.format("%d:%d:%d.%d",hour,min,sec,ms);

    }

    //</editor-fold>
}
