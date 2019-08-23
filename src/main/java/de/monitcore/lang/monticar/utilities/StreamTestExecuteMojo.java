/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;


import de.monitcore.lang.monticar.utilities.tools.ChecksumChecker;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs stream tests executables and checks if their output is valid
 */
@Mojo(name = "streamtest-execute")
public class StreamTestExecuteMojo extends StreamTestMojoBase {
    protected List<EMAComponentSymbol> toRun;
    private boolean throwAMojoExecutionExceptionInPost = false;

    protected String getPathMojoExecutionOutput(){
        return Paths.get(this.getPathTmpOut(), mojoDirectory, this.MojoName(), "output/").toString();
    }

    protected String getExecutable(String name){
        return Paths.get(this.getPathTmpOutBUILD(), name, this.execFileName(name, this.generator)).toAbsolutePath().toString();
    }

    //<editor-fold desc="Execution">

    @Override
    protected void preExecution() throws MojoExecutionException, MojoFailureException {
        super.preExecution();

        toRun = getToTestComponentSymbols(false);


        StreamTestBuildMojo stbm = new StreamTestBuildMojo();
        this.copyPropertiesAndParametersTo(stbm);
        stbm.execute();
    }

    @Override
    protected void mainExecution() throws MojoExecutionException, MojoFailureException {
        super.mainExecution();

        if(failedFileBuild().exists()){
            failedFileBuild().delete();
        }

        this.mkdir(getPathMojoExecutionOutput());
        //FileUtils.cleanDirectory(getPathMojoExecutionOutput());

        List<String> noExecOutput = new ArrayList<>();



        logInfo("Running StreamTest executables:");
        for (EMAComponentSymbol cs : toRun) {
            logInfo(" - "+cs.getFullName());

            if(runExecFor(cs.getFullName())){
                noExecOutput.add("Passed test: "+cs.getFullName());
            }else{
                noExecOutput.add("NOT Passed test: "+cs.getFullName());
                throwAMojoExecutionExceptionInPost = true;
            }

        }

        try {
            FileUtils.writeLines(execFileBuild(), noExecOutput, false);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }



    @Override
    protected void postExecution() throws MojoExecutionException {
        super.postExecution();

        //New hashfile
        String nh = hashOfBuildFiles();
        try {
            FileUtils.write(hashFileBuild(), nh, false);
        } catch (IOException e) {
            e.printStackTrace();
        }

        if(throwAMojoExecutionExceptionInPost){

            try {
                FileUtils.write(failedFileBuild(), "1", false);
            } catch (IOException e) {
                e.printStackTrace();
            }

            throw new MojoExecutionException("One or more tests failed!");
        }

    }

    @Override
    protected boolean checkForExecution() throws MojoExecutionException {
        File hash = hashFileBuild();
        if(!hash.exists()) {
            return true;
        }

        String oldHash = "", newHash = "";
        try{
            oldHash = FileUtils.readFileToString(hash);
            newHash = hashOfBuildFiles();
        } catch (IOException e) {
            e.printStackTrace();
        }

        if(oldHash.equalsIgnoreCase(newHash)){
            logInfo("Streamtest executables not changed:");
            logInfo(" -> Cached output:");
            try {
                List<String> lines = FileUtils.readLines(execFileBuild());
                for(String line : lines){
                    logInfo("    - "+line);
                }
            } catch (IOException e) {
                e.printStackTrace();
            }

            if(failedFileBuild().exists()){
                logError(" -> Cached execution failed. ");

                for (EMAComponentSymbol cs : toRun) {
                    File resultNumber = Paths.get(hashDirBuild().getPath(), cs.getFullName()+".exit.txt").toFile();
                    if(resultNumber.exists())
                    {
                        try {
                            String oen = FileUtils.readFileToString(resultNumber);
                            int exitNumber = Integer.parseInt(oen);
                            if(exitNumber != 0){
                                logError("   -> Test(s) for " + cs.getFullName() + " failed:");
                                List<String> output = FileUtils.readLines(Paths.get(getPathMojoExecutionOutput(), cs.getFullName() + ".exec.out.txt").toFile());
                                for(String line : output){
                                    logError("      [exec-output] "+line);
                                }
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }

                    }
                }

                throw new MojoExecutionException("Cached tests failed and no files changed, so this test also failes!");
            }

            return false;
        }

        return true;
    }

    @Override
    protected String MojoName() {
        return "streamtest-execute";
    }

    //</editor-fold>


    protected boolean runExecFor(String name) throws MojoExecutionException {
        File hash = Paths.get(hashDirBuild().getPath(), name+".txt").toFile();
        File resultNumber = Paths.get(hashDirBuild().getPath(), name+".exit.txt").toFile();
        int oldExitNumber = -1;
        boolean runExec = true;
        String newHash = "";

        String oldHash = "";
        try {
            newHash = ChecksumChecker.getChecksumForFileMD5(getExecutable(name));
            if(hash.exists()){
                oldHash = FileUtils.readFileToString(hash);
                runExec = !newHash.equalsIgnoreCase(oldHash);
                if(resultNumber.exists())
                {
                    String oen = FileUtils.readFileToString(resultNumber);
                    oldExitNumber = Integer.parseInt(oen);
                }else{
                    runExec = true;
                }
            }

        } catch (IOException e) {
            e.printStackTrace();
        }


        int exitNumber = -1;
        File o = Paths.get(getPathMojoExecutionOutput(), name + ".exec.out.txt").toFile();
        if(runExec || forceRun) {
            logInfo("   -> Running "+getExecutable(name));
            //if need to run:
            List<String> command = new ArrayList<>();
            //command.add(this.getExecutable(name));
            if(SystemUtils.IS_OS_WINDOWS){
                //command.add(Paths.get(this.getPathTmpOutBUILD(), name, this.execFileName(name,this.generator)).toString());
                try {
                    FileUtils.copyFile(Paths.get(this.getPathTmpOutBUILD(), name, this.execFileName(name, this.generator)).toFile(),
                            Paths.get(this.getPathTmpOutBUILD(), name, "StreamTests.exe").toFile());
                } catch (IOException e1) {
                    e1.printStackTrace();
                    logError("   -> "+this.execFileName(name, this.generator)+" executable could not be copied. Build failed (unknown error)");
                }
                command.add(Paths.get(this.getPathTmpOutBUILD(), name, "StreamTests.exe").toAbsolutePath().toString());
            }else{
                command.add("./"+this.execFileName(name,this.generator));
            }

            File e = Paths.get(getPathMojoExecutionOutput(), "Exec.err.txt").toFile();
            exitNumber = processRun(command, Paths.get(this.getPathTmpOutBUILD(), name).toString(), o, e, "exec");
            if (exitNumber == 0) {
                logInfo("   -> All tests passed!");
            } else {
                logError("   -> Test(s) for " + name + " failed. See output above");
            }

            try {
                FileUtils.write(resultNumber, Integer.toString(exitNumber), false);
            } catch (IOException e1) {
                e1.printStackTrace();
            }

        }else{

            if (oldExitNumber == 0) {
                logInfo("   -> Unchanged exec file => Using cache: All tests passed!");
            } else {
                logInfo("   -> Unchanged => Using cache:");
                logError("      -> Cached exit code: "+oldExitNumber);
                logError("      -> Test(s) for " + name + " failed:");
                try {
                    List<String> output = FileUtils.readLines(o);
                    for(String line : output){
                        logError("      [exec-output] "+line);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }
            exitNumber = oldExitNumber;
        }

        try {
            FileUtils.write(hash, newHash, false);
        } catch (IOException e) {
        }

        return exitNumber == 0;
    }


    protected String hashOfBuildFiles(){
        StringBuilder sb = new StringBuilder();
        for (EMAComponentSymbol cs : toRun) {
            try {
                sb.append(ChecksumChecker.getChecksumForFileMD5(getExecutable(cs.getFullName())));
            } catch (IOException e) {
            }
        }
        return sb.toString();
    }


    //<editor-fold desc="Hashfiles">

    protected File hashDirBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "build").toFile();
    }

    protected File hashFileBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "build.txt").toFile();
    }

    protected File execFileBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "exec.txt").toFile();
    }
    protected File failedFileBuild(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "failed.txt").toFile();
    }

    //</editor-fold>
}
