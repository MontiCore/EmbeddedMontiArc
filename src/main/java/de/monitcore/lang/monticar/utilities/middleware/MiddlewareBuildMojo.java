/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.middleware;

import de.monitcore.lang.monticar.utilities.tools.LogToFile;
import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * runs cmake and make for generated c++ code of given middlewarerootmodels
 */
@Mojo(name = "streamtest-middleare-build")
public class MiddlewareBuildMojo extends MiddlewareMojoBase {


    @Override
    protected void preExecution() throws MojoExecutionException, MojoFailureException {

        mkdir(Paths.get(this.getPathMiddlewareOut(), mojoDirectory).toString());
        mkdir(Paths.get(this.getPathMiddlewareOut(), mojoDirectory, this.MojoName()).toString());
        mkdir(Paths.get(this.getPathMiddlewareOut(), mojoDirectory, this.MojoName(), "hashes").toString());
        mkdir(Paths.get(this.getPathMiddlewareOut(), mojoDirectory, this.MojoName(), "runbuild").toString());

        if(myLog == null) {
            myLog = LogToFile.init();
        }
        myLog.setLogFile(Paths.get(this.getPathMiddlewareOut(), mojoDirectory, this.MojoName()+".log").toString());
        myLog.clear();

        if(SystemUtils.IS_OS_WINDOWS){
            Log.error("Mojo streamtest-middleare-build only works on Linux & MacOS. For more information look at the README of EMAM2Middleware.");
            throw new MojoExecutionException("Mojo streamtest-middleare-build only works on Linux & MacOS. For more information look at the README of EMAM2Middleware.");
        }



        logInfo("Running Middleware Generator Mojo before building Middleware!");
        MiddlewareGeneratorMojo stgm = new MiddlewareGeneratorMojo();
        this.copyPropertiesAndParametersTo(stgm);
        stgm.execute();
    }

    @Override
    protected void mainExecution() throws MojoExecutionException, MojoFailureException {

        List<String> modelsInError = new ArrayList<>();
        logInfo("Build generated Middleware:");
        boolean allCompiled = true;
        for (String model : this.middlewareRootModels){
            logInfo(" - "+model);
            String modelDirName = model.replace(".", "_");
            if(checkHashesForBuild(modelDirName)){
                logInfo("   -> Files are up to date");
                continue;
            }
            allCompiled = allCompiled && build(model, modelDirName);
        }
        if(!allCompiled){
            throw new MojoFailureException("Not all models were builded");
        }
    }

    @Override
    protected void postExecution() throws MojoExecutionException {
        super.postExecution();


        String buildHash = SearchFiles.hashDirFiles(this.getPathTmpOutBUILD());
        try {
            if(hashDirBuild().exists()){
                FileUtils.deleteDirectory(hashDirBuild());
            }
            this.mkdir(this.hashDirBuild().toString());

        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Failed to create hash files for "+MojoName());
        }


        for (String model : this.middlewareRootModels){
            String modelDirName = model.replace(".", "_");
            buildHash = SearchFiles.hashDirFiles(Paths.get(this.pathMiddlewareOut, modelDirName).toString());
            try {
                FileUtils.write(hashFileBuildFor(modelDirName), buildHash, false);
            } catch (IOException e) {
                e.printStackTrace();
                throw new MojoExecutionException("Failed to create hash files for "+MojoName());
            }
        }
    }

    @Override
    protected String MojoName() {
        return "streamtest-middleare-build";
    }


    protected boolean build(String model, String modelDirName) throws MojoExecutionException {
        logInfo("   -> Build ");

        logInfo("   -> Creating build/ dir");
        Path buildPath = Paths.get(this.pathMiddlewareOut, modelDirName, "build/");
        this.mkdir(buildPath.toString());
        try {
            FileUtils.cleanDirectory(buildPath.toFile());
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException(e.getMessage());
        }

        List<String> command = new ArrayList<>();
        command.add("cmake");
        command.add("../src");

        logInfo("   # Run ("+buildPath.toString()+"): "+String.join(" ", command)+" ");
        long startTime = System.nanoTime();
        File o = Paths.get(processTmpDir(), model+".cmake.out.txt").toFile();
        File e = Paths.get(processTmpDir(), model+".cmake.err.txt").toFile();
        if(processRun(command, buildPath.toAbsolutePath().toString(), o,e, "build")==0){
            logInfo("     -> Success in "+getReadableTime(System.nanoTime()-startTime));
        }else{
            logInfo("     -> Error. CMake failed for "+model);
            return false;
        }



        command.clear();
        command.add("make");
        logInfo("   # Run ("+buildPath.toString()+"): "+String.join(" ", command)+" ");
        startTime = System.nanoTime();
        o = Paths.get(this.processTmpDir(), model+".build.out.txt").toFile();
        e = Paths.get(this.processTmpDir(), model+".build.err.txt").toFile();
        if(processRun(command, buildPath.toAbsolutePath().toString(),o,e, "build")==0){
            logInfo("     -> Success in "+getReadableTime(System.nanoTime()-startTime));
        }else{
            logInfo("     -> Error. Make BUILD failed for "+model);
            return false;
        }


        return true;
    }


    protected boolean checkHashesForBuild(String dirName){
        if(forceRun){
            return false;
        }
        File hbuild = hashFileBuildFor(dirName);
        if(!hbuild.exists()){
            return false;
        }

        String oldBUILDHash = "";
        String newBUILDHash = SearchFiles.hashDirFiles(Paths.get(this.pathMiddlewareOut, dirName).toString());
        try {
            oldBUILDHash = FileUtils.readFileToString(hbuild);
        }catch (Exception ex){
            return false;
        }
        if(newBUILDHash.equalsIgnoreCase(oldBUILDHash) ){
            return true;
        }

        return false;
    }

    protected File hashDirBuild(){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "hashes").toFile();
    }

    protected File hashFileBuildFor(String dirName){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "hashes", dirName+".txt").toFile();
    }

    protected String processTmpDir(){
        return Paths.get(this.pathMiddlewareOut, mojoDirectory, this.MojoName(), "runbuild").toString();
    }
}
