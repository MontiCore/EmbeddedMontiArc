/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;

import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.*;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * builds the executable stream test out of the generated c++ code for testable components
 */
@Mojo(name = "streamtest-build")
public class StreamTestBuildMojo extends StreamTestMojoBase {




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

        List<EMAComponentSymbol> toBuild = getToTestComponentSymbols(false);
        for (EMAComponentSymbol cs : toBuild) {
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

        List<EMAComponentSymbol> toBuild = getToTestComponentSymbols(true);


        logInfo("Build StreamTest executables:");

        boolean allCompiled = true;

        for (EMAComponentSymbol cs : toBuild) {
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
        if(forceRun){
            return false;
        }
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

            if(Paths.get(this.getPathTmpOutBUILD(), name, execFileName(name, this.generator)).toFile().exists()){
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
        String targetname = this.fullNameToCMakeTarget(name);
        if(SystemUtils.IS_OS_MAC){
            logInfo("   -> Set MacOS CMake workaround for streamtests building");
            this.setCMakeMacOS(name, targetname);
        }

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
//        command.add("StreamTests");
//        command.add(name.replace(".", "_")+"_StreamTests");
        command.add(targetname);
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

        if(!Paths.get(this.getPathTmpOutBUILD(), name, this.execFileName(name, this.generator)).toFile().exists()){
            logError("   -> StreamTests executable was not found. Build failed (unknown error)");
        }

        if(SystemUtils.IS_OS_MAC){
            logInfo("   -> Unset MacOS CMake workaround");
            this.unsetCMakeMacOS(name, targetname);
        }

        return true;
    }

    private void setCMakeMacOS(String name, String targetname) throws MojoExecutionException {
        try {
            List<String> lines = FileUtils.readLines(Paths.get(this.getPathTmpOutCPP(), name, "CMakeLists.txt").toFile());
            //target_link_libraries(de_rwth_cmplib_basicLibrary_or_TestWrapper_StreamTests
            //set_target_properties(de_rwth_cmplib_basicLibrary_or_TestWrapper_StreamTests
            String starget = "target_link_libraries("+targetname;
            String sset = "set_target_properties("+targetname;
            for(int i = 0; i < lines.size(); ++i){
                String line = lines.get(i);
                if(line.startsWith(sset) || line.startsWith(starget)){
                    lines.set(i, "# "+line);
                }
            }
            FileUtils.writeLines(Paths.get(this.getPathTmpOutCPP(), name, "CMakeLists.txt").toFile(), lines, false);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Error while chaning CMakeLists.txt for macos workaround");
        }
    }

    private void unsetCMakeMacOS(String name, String targetname) throws MojoExecutionException {
        try {
            List<String> lines = FileUtils.readLines(Paths.get(this.getPathTmpOutCPP(), name, "CMakeLists.txt").toFile());
            String starget = "# target_link_libraries("+targetname;
            String sset = "# set_target_properties("+targetname;
            for(int i = 0; i < lines.size(); ++i){
                String line = lines.get(i);
                if(line.startsWith(sset) || line.startsWith(starget)){
                    lines.set(i, line.substring(2));
                }
            }
            FileUtils.writeLines(Paths.get(this.getPathTmpOutCPP(), name, "CMakeLists.txt").toFile(), lines, false);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Error while chaning CMakeLists.txt for macos workaround");
        }
    }

    private void runTraining(){
        Optional<Backend> backend;
        backend = Backend.getBackendFromString("Gluon");
        EMADLGenerator emadlGenerator = new EMADLGenerator(backend.get());
        try{
            emadlGenerator.generate(getPathMain(), "VGG16", "pythonPath", "y", true, "n");
        }
        catch (IOException | TemplateException e){
            Log.error("io error during generation", e);
            System.exit(1);
        }



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

