/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;

import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
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
import java.util.Map;

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

        List<EMAComponentSymbol> toBuild = getToTestComponentSymbols(false);


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
        if(processRun(command, buildDir, o, e, "build")==0){
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

    protected boolean runTraining() throws MojoExecutionException, IOException {
        if (!checkJarArchiveForDL()){
            throw new MojoExecutionException("No training archive!");
        }
        String buildTarget = Paths.get(this.getPathTmpOut()).toString();

        //check the location of DL framework mxnet.
        File tempScript = createTempScript();
        try {
            ProcessBuilder pb = new ProcessBuilder("bash", tempScript.toString());
            pb.inheritIO();
            Process process = pb.start();
            int returnCode = process.waitFor();
            if(returnCode != 0) {
                Log.error("During compilation, an error occured. See above for more details.");
                System.exit(1);
            }
        }catch(Exception e){
            Log.error("During compilation, the following error occured: '" + e.toString() + "'");
            System.exit(1);
        } finally {
            tempScript.delete();
        }

        //Run EMADL generator:
        List<String> command = new ArrayList<>();
        command.add("java -jar  embedded-montiarc-emadl-generator-0.3.6-jar-with-dependencies.jar");
        command.add("-m model");
        command.add("-r VGG16");
        command.add("-o " + buildTarget);
        command.add("-b MXNET");

        logInfo("   # Run Training with background MXNet ");
        long startTime = System.nanoTime();
        File o = Paths.get(this.runProcessTMPDir().toString(), ".build.out.txt").toFile();
        File e = Paths.get(this.runProcessTMPDir().toString(), ".build.err.txt").toFile();

        if(processRun(command, "./",o,e, "Train")==0){
            logInfo("     -> Success in "+getReadableTime(System.nanoTime()-startTime));
        }else{
            logInfo("     -> Error. Training failed");
            return false;
        }

        return true;
    }


    public File createTempScript() throws IOException{
        File tempScript = File.createTempFile("script", null);
        try{
            Writer streamWriter = new OutputStreamWriter(new FileOutputStream(
                    tempScript));
            PrintWriter printWriter = new PrintWriter(streamWriter);

            printWriter.println("MXNET_PATH=$(python -c \"import mxnet; print(mxnet.__file__)\")");
            printWriter.println("# (c) https://github.com/MontiCore/monticore  ");
            printWriter.println("MXNET_FOLDER=$(dirname $MXNET_PATH)");
            printWriter.println("echo $MXNET_FOLDER");

            printWriter.println("if [ ! -f $MXNET_FOLDER/libmxnet.so ]; then");
            printWriter.println("echo \"libmxnet.so not found at default location\" $MXNET_FOLDER");
            printWriter.println("echo \"It should be there if the python mxnet package is installed\"");
            printWriter.println("echo \"Either fix the installation, or adapt the ./build.sh script to locate libmxnet.so correctly\"");
            printWriter.println("exit 1");
            printWriter.println("fi");

            printWriter.close();
        }catch(Exception e){
            System.out.println(e);
        }

        return tempScript;
    }


    private boolean checkJarArchiveForDL() throws MojoExecutionException{
        Map<String, File> files = SearchFiles.searchFilesMap("./", "jar");
        for (Map.Entry<String,File> f:files.entrySet()) {
            String ending = f.getKey().substring(f.getKey().lastIndexOf(".") + 1);
            if (ending.equals("jar")){
                return true;
            }
        }
        return false;

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



    //Path in form ./{PATHTMPOUT}/{mojoDirectory}/{streamtest-build}/build
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
