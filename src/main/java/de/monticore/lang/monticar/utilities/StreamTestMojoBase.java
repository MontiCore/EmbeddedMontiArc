/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.enumlang._parser.EnumLangParser;
import de.monticore.lang.monticar.streamunits._parser.StreamUnitsParser;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.utilities.utils.LogToFile;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;

public class StreamTestMojoBase extends BaseMojo {

  @Parameter(name = "wrapperTestExtension", defaultValue = "_TestWrapper")
  protected String wrapperTestExtension;
  public String getWrapperTestExtension() {
    return wrapperTestExtension;
  }
  public void setWrapperTestExtension(String wrapperTestExtension) {
    this.wrapperTestExtension = wrapperTestExtension;
  }

  @Parameter(defaultValue = "false")
  protected boolean forceRun;
  public boolean isForceRun() {
    return forceRun;
  }
  public void setForceRun(boolean forceRun) {
    this.forceRun = forceRun;
  }

  @Parameter(defaultValue = "true")
  protected boolean showDateAndTime;

  protected static String mojoDirectory = "mojo/";

  protected static LogToFile myLog = null;

  @Override
  public void execute() throws MojoExecutionException, MojoFailureException {
    Log.enableFailQuick(false);
    this.preExecution();
    if(forceRun || checkForExecution()) {
      if(forceRun){
        logInfo(this.MojoName()+" is forced to run!");
      }
      this.mainExecution();
      this.postExecution();
    }
  }

  protected void preExecution() throws MojoExecutionException, MojoFailureException {
    this.mkdir(this.getPathTmpOut());
    this.mkdir(this.getPathTmpOutCPP());
    this.mkdir(this.getPathTmpOutEMAM());
    this.mkdir(this.getPathTmpOutEMADL());
    this.mkdir(this.getPathTmpOutBUILD());
    this.mkdir(Paths.get(this.getPathTmpOut(), mojoDirectory).toString());
    this.mkdir(Paths.get(this.getPathTmpOut(), mojoDirectory, this.MojoName()).toString());

    Log.enableFailQuick(false);

    if(myLog == null) {
      myLog = LogToFile.init();
    }
    myLog.setLogFile(Paths.get(this.getPathTmpOut(), mojoDirectory, this.MojoName()+".log").toString());
    myLog.clear();
  }

  protected  void mainExecution() throws MojoExecutionException, MojoFailureException {
    // tada
  }

  protected void postExecution() throws MojoExecutionException {
    // hier koennte ihre werbung stehen
  }

  protected boolean checkForExecution() throws MojoExecutionException {
    return true;
  }


  protected String MojoName(){
    return "StreamTestMojoBase";
  }

  protected static void resetToMyLog(){
    if(myLog == null){
      myLog = LogToFile.init();
    }else{
      LogToFile.resetTo(myLog);
    }
  }

  protected void logError(String msg){
    if(showDateAndTime){
      msg = addDateTime(msg);
    }
    getLog().error(msg);
    Log.error(msg);
  }

  protected void logInfo(String msg){
    if(showDateAndTime){
      msg = addDateTime(msg);
    }

    getLog().info(msg);
    Log.info(msg, this.MojoName());
  }

  protected void logWarn(String msg){
    if(showDateAndTime){
      msg = addDateTime(msg);
    }
    getLog().warn(msg);
    Log.warn(msg);
  }

  protected void logDebug(String msg){
    if(showDateAndTime){
      msg = addDateTime(msg);
    }
    getLog().debug(msg);
    Log.debug(msg, this.MojoName());
  }

  public String getCurrentLocalDateTimeStamp() {
    return LocalDateTime.now()
        .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS"));
  }

  public String addDateTime(String msg){
    return String.format("[%s] %s", getCurrentLocalDateTimeStamp(), msg);
  }

  protected void mkdir(String path) throws MojoExecutionException {
    try {
      File tmpOut = Paths.get(path).toFile();
      if(tmpOut.exists() && tmpOut.isFile()){
        throw new MojoExecutionException("path:(\""+path+"\") is a file!");
      }
      if(!tmpOut.exists()){
        tmpOut.mkdirs();
      }
    }catch (Exception ex){
      ex.printStackTrace();
      throw new MojoExecutionException(ex.getMessage());
    }
  }

  private Map<String,MCConcreteParser> myParser = null;
  protected Map<String,MCConcreteParser> getParser(){
    if(myParser == null) {
      myParser = new HashMap<>();
      myParser.put("emam", new EmbeddedMontiArcMathParser());
      myParser.put("stream", new StreamUnitsParser());
      myParser.put("struct", new StructParser());
      myParser.put("enum", new EnumLangParser());
      myParser.put("emadl",new EMADLParser());
      resetToMyLog();
    }
    return myParser;
  }

  protected List<EMAComponentSymbol> getComponentSymbols(boolean output) {
    ComponentScanner componentScannerEMADL = new ComponentScanner(Paths.get(this.getPathMain()), this.getScope(), "emadl");
    Set<String> componentNames = componentScannerEMADL.scan();

    ComponentScanner componentScannerEMAM = new ComponentScanner(Paths.get(this.getPathMain()), this.getScope(), "emam");
    componentNames.addAll(componentScannerEMAM.scan());

    StreamScanner scanner = new StreamScanner(Paths.get(this.getPathMain()), this.getScope());
    Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> streamTests = scanner.scan();

    if(output) {
      logInfo("Searching components (with wrapper) with streamtests.");
    }

    List<EMAComponentSymbol> toTestComponents = new ArrayList<>();
    for (String componentName: componentNames) {
      if(output){
        logInfo(" - "+componentName);
      }
      Optional<EMAComponentSymbol> cs = getTestComponentSymbol(componentName, streamTests.keySet());
      if(!cs.isPresent()){
        if(output) {
          logWarn("   -> No streamtest found for " + componentName);
        }
      }else{
        if(output){
          logInfo("   -> Test with component: "+cs.get().getFullName()+" :");
          streamTests.get(cs.get()).forEach( (ComponentStreamUnitsSymbol csus) -> logInfo("      # Streamtest: "+csus.getFullName()));
        }
        toTestComponents.add(cs.get());
      }
    }

    return toTestComponents;
  }

  protected Optional<EMAComponentSymbol> getTestComponentSymbol(String componentName, Set<EMAComponentSymbol> componentSymbols){

    Optional<EMAComponentSymbol> cs = FindComponentSymbolByName(componentName, componentSymbols);
    if(!cs.isPresent()){
      cs = FindComponentSymbolByName(componentName+this.wrapperTestExtension, componentSymbols);
    }
    return cs;
  }

  protected Optional<EMAComponentSymbol> FindComponentSymbolByName(String name, Set<EMAComponentSymbol> componentSymbols){
    for (EMAComponentSymbol cs :componentSymbols) {
      if(cs.getFullName().equals(name)){
        return Optional.of(cs);
      }
    }

    return Optional.empty();
  }

}
