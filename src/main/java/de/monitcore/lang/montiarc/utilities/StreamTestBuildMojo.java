package de.monitcore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.tools.SearchFiles;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.*;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

@Mojo(name = "streamtest-build")
public class StreamTestBuildMojo extends StreamTestMojoBase {

/*
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
*/
    @Override
    protected void preExecution() throws MojoExecutionException, MojoFailureException {
        super.preExecution();
        this.mkdir(this.hashDirBuild().toString());
        this.mkdir(this.hashDirCPP().toString());


        StreamTestGeneratorMojo stgm = new StreamTestGeneratorMojo();
        this.copyPropertiesAndParametersTo(stgm);
        stgm.execute();


    }

    @Override
    protected void mainExecution() throws MojoExecutionException, MojoFailureException {

        Log.info("StreamTestBuildMojo", "StreamTestBuildMojo");

        this.removeMojoFiles();
        this.mkdir(this.runProcessTMPDir().toString());

        if (SystemUtils.IS_OS_WINDOWS) {
            logInfo("Building with cmake for: \""+this.generator+"\"");
        }else{
            logInfo("Building with cmake for: make");
        }


        buildStreamTests();

    }

    @Override
    protected void postExecution() throws MojoExecutionException {
        super.postExecution();


        String buildHash = SearchFiles.hashDirFiles(this.getPathTmpOutBUILD());
        String cppHash = SearchFiles.hashDirFiles(this.getPathTmpOutCPP());
        try {
            if(hashDirCPP().exists()){
                FileUtils.deleteDirectory(hashDirCPP());
            }
            if(hashDirBuild().exists()){
                FileUtils.deleteDirectory(hashDirBuild());
            }
            this.mkdir(this.hashDirBuild().toString());
            this.mkdir(this.hashDirCPP().toString());

            FileUtils.write(hashFileBuild(), buildHash, false);
            FileUtils.write(hashFileCPP(), cppHash, false);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Failed to create hash files for "+MojoName());
        }

        List<ComponentSymbol> toBuild = getToTestComponentSymbols(false);
        for (ComponentSymbol cs : toBuild) {
            buildHash = SearchFiles.hashDirFiles(Paths.get(this.getPathTmpOutBUILD(), cs.getFullName()).toString());
            cppHash = SearchFiles.hashDirFiles(Paths.get(this.getPathTmpOutCPP(), cs.getFullName()).toString());
            try {
                FileUtils.write(hashFileBuildFor(cs.getFullName()), buildHash, false);
                FileUtils.write(hashFileCPPFor(cs.getFullName()), cppHash, false);
            } catch (IOException e) {
                e.printStackTrace();
                throw new MojoExecutionException("Failed to create hash files for "+MojoName());
            }
        }
    }

    @Override
    protected String MojoName() {
        return "streamtest-build";
    }


    //<editor-fold desc="Buildfile creation">


    protected void getGeneratorString(GeneratorEnum gen, List<String> cmd){
        switch (gen){
            case MinGW:
                cmd.add("-G");
                cmd.add("\"MinGW Makefiles\"");
                break;
                //return "-G \"MinGW Makefiles\"";
            case VS2017:
            case VisualStudio2017:
                cmd.add("-G");
                cmd.add("\"Visual Studio 15 2017 Win64\"");
                break;
                //return "-G "Visual Studio 15 2017 Win64";
        }
    }

    protected void getGeneratorBuildOptions(GeneratorEnum gen, List<String> cmd){
        switch (gen){
            case VS2017:
            case VisualStudio2017:
                cmd.add("--config");
                cmd.add("Debug");
                break;
                //return "--config Debug";
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
            if(checkHashesAndExecForBuild(cs.getFullName())){
                logInfo("   -> Files are up to date");
                continue;
            }
            allCompiled = allCompiled && buildTestExecutable(cs.getFullName());
        }

        if(!allCompiled){
            throw new MojoFailureException("Not all streamtest were builded");
        }
    }

    protected boolean checkHashesAndExecForBuild(String name){
        File hbuild = hashFileBuildFor(name);
        File hcpp = hashFileCPPFor(name);
        if(!hbuild.exists() || !hcpp.exists()){
            return false;
        }

        String oldCPPHash = "", oldBUILDHash = "";
        String newCPPHash = SearchFiles.hashDirFiles(Paths.get(this.getPathTmpOutCPP(), name).toString());
        String newBUILDHash = SearchFiles.hashDirFiles(Paths.get(this.getPathTmpOutBUILD(), name).toString());
        try {
            oldBUILDHash = FileUtils.readFileToString(hbuild);
            oldCPPHash = FileUtils.readFileToString(hcpp);
        }catch (Exception ex){
            return false;
        }
        if(newCPPHash.equalsIgnoreCase(oldCPPHash) && newBUILDHash.equalsIgnoreCase(oldBUILDHash) ){

            if(Paths.get(this.getPathTmpOutBUILD(), name, execFileName(this.generator)).toFile().exists()){
                return true;
            }

            return false;
        }

        return false;
    }


    protected boolean buildTestExecutable(String name) throws MojoExecutionException{
        //long startTime = System.nanoTime();
        String buildDir = Paths.get(this.getPathTmpOutBUILD(), name).toString();
        File bd = Paths.get(buildDir).toFile();
        if(bd.exists()){
            try {
                FileUtils.deleteDirectory(bd);
            } catch (IOException e) {
                e.printStackTrace();
                throw new MojoExecutionException("Cannot delete build directory for "+name);
            }
        }
        this.mkdir(buildDir);

        String relPath = Paths.get(this.getPathTmpOutBUILD(),name).relativize(Paths.get(this.getPathTmpOutCPP(), name)).toString();

        //Run cmake:
        List<String> command = new ArrayList<>();
        command.add("cmake");
        command.add(""+relPath+"");
        if (SystemUtils.IS_OS_WINDOWS) {
            //command.add(this.getGeneratorString(this.getGenerator()));
            this.getGeneratorString(this.generator, command);
        }
        logInfo("   # Run ("+buildDir+"): "+String.join(" ", command)+" ");
        long startTime = System.nanoTime();
        File o = Paths.get(this.runProcessTMPDir().toString(), name+".cmake.out.txt").toFile();
        File e = Paths.get(this.runProcessTMPDir().toString(), name+".cmake.err.txt").toFile();
        if(processRun(command, buildDir, o,e, "build")==0){
            logInfo("     -> Success in "+getReadableTime(System.nanoTime()-startTime));
        }else{
            logInfo("     -> Error. CMake failed for "+name);
            return false;
        }

        command.clear();
        command.add("cmake");
        command.add("--build");
        command.add(".");
        command.add("--target");
        command.add("StreamTests");
        if (SystemUtils.IS_OS_WINDOWS) {
            //command.add(this.getGeneratorString(this.getGenerator()));
            this.getGeneratorBuildOptions(this.generator, command);
        }
        logInfo("   # Run ("+buildDir+"): "+String.join(" ", command)+" ");
        startTime = System.nanoTime();
        o = Paths.get(this.runProcessTMPDir().toString(), name+".build.out.txt").toFile();
        e = Paths.get(this.runProcessTMPDir().toString(), name+".build.err.txt").toFile();
        if(processRun(command, buildDir,o,e, "build")==0){
            logInfo("     -> Success in "+getReadableTime(System.nanoTime()-startTime));
        }else{
            logInfo("     -> Error. CMake BUILD failed for "+name);
            return false;
        }

        if(!Paths.get(this.getPathTmpOutBUILD(), name, this.execFileName(this.generator)).toFile().exists()){
            logError("   -> StreamTests executable was not found. Build failed (unknown error)");
        }

        return true;
    }

    private static String getReadableTime(Long nanos){

        long tempSec = nanos/(1000*1000*1000);
        long ms = (nanos/(1000*1000))%1000;
        long sec = tempSec % 60;
        long min = (tempSec /60) % 60;
        long hour = (tempSec /(60*60));
        return String.format("%02d:%02d:%02d.%03d",hour,min,sec,ms);

    }

    //</editor-fold>

    //<editor-fold desc="Hashfiles">

    protected void removeMojoFiles() throws MojoExecutionException {
        try {
            /*if(hashDirCPP().exists()){
                FileUtils.deleteDirectory(hashDirCPP());
            }
            if(hashDirBuild().exists()){
                FileUtils.deleteDirectory(hashDirBuild());
            }*/
            if(hashFileBuild().exists()){
                hashFileBuild().delete();
            }
            if(hashFileCPP().exists()){
                hashFileCPP().delete();
            }
            if(runProcessTMPDir().exists()){
                FileUtils.deleteDirectory(runProcessTMPDir());
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Cannot delete tmp mojo directorys!");
        }
    }

    protected File hashDirBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "build").toFile();
    }

    protected File hashFileBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "build.txt").toFile();
    }

    protected File hashFileBuildFor(String name){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "build/", name+".txt").toFile();
    }

    protected File hashDirCPP(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "cpp").toFile();
    }

    protected File hashFileCPP(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "cpp.txt").toFile();
    }

    protected File hashFileCPPFor(String name){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "cpp/", name+".txt").toFile();
    }

    protected File runProcessTMPDir(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "run").toFile();
    }
    //</editor-fold>
}
