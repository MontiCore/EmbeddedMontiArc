/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.monticar.utilities.utils.SearchFiles;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Execute;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

/**
 * Runs CoCos test on all components in pathMain and pathTest
 * Generates c++ code for all components which have a stream test
 */
@Mojo(name = "start-training")
@Execute(goal = "validate")
public class TrainingMojo extends StreamTestMojoBase {

  @Parameter
  private TrainingConfiguration training;


  @Override
  protected void preExecution() throws MojoExecutionException, MojoFailureException {
    super.preExecution();

  }

  @Override
  protected void mainExecution() throws MojoExecutionException {
    Log.info("StreamTestGeneratorMojo", "StreamTestGeneratorMojo");

    //Remove old hasfiles
    File fmain = hashFileMain();
    if(fmain.exists()){
      fmain.delete();
    }
    File ftest = hashFileTest();
    if(ftest.exists()){
      ftest.delete();
    }
    File femam = hashEmamFile();
    if(femam.exists()){
      femam.delete();
    }
    File femadl = hashEmadlFile();
    if(femadl.exists()){
      femadl.delete();
    }

    try{
      File temam = Paths.get(this.getPathTmpOutEMAM()).toFile();
      FileUtils.copyDirectory(Paths.get(this.pathMain).toFile(), temam);
      FileUtils.copyDirectory(Paths.get(this.pathTest).toFile(), temam);
    } catch (IOException e) {
      e.printStackTrace();
      throw new MojoExecutionException("Could not copy files: "+e.getMessage() );
    }

    train();
  }

  @Override
  protected void postExecution() throws MojoExecutionException {
    super.postExecution();
    //Create hash of files
    String mainHash = SearchFiles.hashDirFiles(this.getPathMain());
    String testHash = SearchFiles.hashDirFiles(this.getPathTest());
    String emamHash = SearchFiles.hashDirFiles(this.getPathTmpOutEMAM());
    try {
      FileUtils.write(hashFileMain(), mainHash, false);
      FileUtils.write(hashFileTest(), testHash, false);
      FileUtils.write(hashEmamFile(), emamHash, false);
    } catch (IOException e) {
      e.printStackTrace();
      throw new MojoExecutionException("Failed to create hash files for "+MojoName());
    }
  }

  @Override
  protected boolean checkForExecution() throws MojoExecutionException {
    File hmain = hashFileMain();
    File htest = hashFileTest();
    File hemam = hashEmamFile();
    if(!hmain.exists() || !htest.exists() || !hemam.exists() ){
      logInfo("Execution necessary: Hashfiles not found.");
      return true;
    }

    String oldMainHash = "",oldTestHash = "", oldEmamHash = "", oldCppHash;

    String newMainHash = SearchFiles.hashDirFiles(this.getPathMain());
    String newTestHash = SearchFiles.hashDirFiles(this.getPathTest());
    String newEmamHash = SearchFiles.hashDirFiles(this.getPathTmpOutEMAM());

    try {
      oldMainHash = FileUtils.readFileToString(hmain);
      oldTestHash = FileUtils.readFileToString(htest);
      oldEmamHash = FileUtils.readFileToString(hemam);
    } catch (IOException e) {
      e.printStackTrace();
      throw new MojoExecutionException("Can't read old hash files");
    }

    if(newMainHash.equalsIgnoreCase(oldMainHash) && newTestHash.equalsIgnoreCase(oldTestHash) &&
        newEmamHash.equalsIgnoreCase(oldEmamHash) ){
      logInfo("Execution of "+this.MojoName().toUpperCase()+" not necessary. No input files changed.");
      return false;
    }

    logInfo("Execution necessary: One or more input files chaned.");
    return true;
  }


  public void train() {
    EMADLGenerator emadlGenerator = new EMADLGenerator(training.getBackend());

    String outputPath = getPathTmpOut();
    if (outputPath != null){
      emadlGenerator.setGenerationTargetPath(outputPath);
    }
    try{
      emadlGenerator.generate(this.getPathMain(), training.getModelToTrain(), training.getPathToPython().getAbsolutePath(), "x", true);
    }
    catch (IOException e){
      Log.error("io error during generation", e);
      System.exit(1);
    }
    catch (TemplateException e){
      Log.error("template error during generation", e);
      System.exit(1);
    }
  }


  //<editor-fold desc="Hashfiles">

  protected File hashFileMain(){
    return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "Main.txt").toFile();
  }

  protected File hashFileTest(){
    return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "Test.txt").toFile();
  }

  protected File hashEmamFile(){
    return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "Emam.txt").toFile();
  }

  protected File hashEmadlFile(){
    return Paths.get(this.pathTmpOut,mojoDirectory,this.MojoName(),"Emadl.txt").toFile();
  }

  //</editor-fold>

  //<editor-fold desc="Utilities">
  protected String modelNameCalculator(File f, String ending,  List<String> packages){
    String packageName = "";
    if(!packages.isEmpty()) {
      packageName = Joiners.DOT.join(packages) + ".";
    }
    return packageName+f.getName().replace("."+ending, "");
  }
  //</editor-fold>
}
