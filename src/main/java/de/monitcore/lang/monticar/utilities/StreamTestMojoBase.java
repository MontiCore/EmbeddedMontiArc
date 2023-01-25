/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;

import de.monitcore.lang.monticar.utilities.tools.LogToFile;
import de.monticore.ModelingLanguageFamily;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.emadl.generator.Backend;
import de.monticore.lang.monticar.enumlang._parser.EnumLangParser;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._parser.StreamUnitsParser;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.nio.charset.Charset;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;

public class StreamTestMojoBase extends AbstractMojo {

    //<editor-fold desc="Parameter">

    @Parameter(property = "pathMain",defaultValue = "./src/main/emam/")
    protected String pathMain;
    public String getPathMain() {
        return pathMain;
    }
    public void setPathMain(String path) {
        this.pathMain = path;
    }

    @Parameter(property = "pathTest",defaultValue = "./src/test/emam/")
    protected String pathTest;
    public String getPathTest() {
        return pathTest;
    }
    public void setPathTest(String pathTest) {
        this.pathTest = pathTest;
    }

    @Parameter(property = "pathTmpOut", defaultValue = "./target/tmp")
    protected String pathTmpOut;
    public String getPathTmpOut() {
        return pathTmpOut;
    }
    public void setPathTmpOut(String pathTmpOut) {
        this.pathTmpOut = pathTmpOut;
    }
    public String getPathTmpOutCPP(){
        return Paths.get(this.pathTmpOut, "cpp/").toString();
    }
    public String getPathTmpOutEMAM(){
        return Paths.get(this.pathTmpOut, "emam/").toString();
    }
    public String getPathTmpOutEMADL(){
        return Paths.get(this.pathTmpOut,"emadl/").toString();
    }
    public String getPathTmpOutBUILD() {
        return Paths.get(this.getPathTmpOut(), "/").toString();
    }

    @Parameter(name = "wrapperTestExtension", defaultValue = "_TestWrapper")
    protected String wrapperTestExtension;
    public String getWrapperTestExtension() {
        return wrapperTestExtension;
    }
    public void setWrapperTestExtension(String wrapperTestExtension) {
        this.wrapperTestExtension = wrapperTestExtension;
    }

    @Parameter(defaultValue = "NONE")
    protected GeneratorEnum generator;
    public GeneratorEnum getGenerator() {
        return generator;
    }
    public void setGenerator(GeneratorEnum generator) {
        this.generator = generator;
    }

    @Parameter(defaultValue = "GLUON")
    protected Backend backend;
    public Backend getBackend(){ return backend;}
    public void setBackend(Backend backend){ this.backend = backend;}

    @Parameter(defaultValue = "/usr/bin/python")
    protected String pathToPython;
    public String getPathToPython() {return pathToPython;}
    public void setPathToPython(String pathToPython){this.pathToPython = pathToPython;}

    @Parameter(defaultValue = "")
    protected String customFilesPath;
    public String getCustomFilesPath() {return customFilesPath;}
    public void setCustomFilesPath(String customFilesPath){this.customFilesPath = customFilesPath;}

    @Parameter(defaultValue = "n")
    protected String useDgl;
    public String getUseDgl() {return useDgl;}
    public void setUseDgl(String useDgl){this.useDgl = useDgl;}

    @Parameter(defaultValue = "VGG16")
    protected String rootModel;
    public String getRootModel() { return rootModel;}
    public void setRootModel(String rootModel){this.rootModel = rootModel;}

    @Parameter(defaultValue = "false")
    protected boolean trainingNeeded;
    public boolean getTrainingNeeded(){ return trainingNeeded;}
    public void setTrainingNeeded(boolean trainingNeeded){ this.trainingNeeded = trainingNeeded;}

    @Parameter(defaultValue = "false")
    protected boolean combinebuilds;
    public boolean getCombinebuilds() {
        return combinebuilds;
    }
    public void setCombinebuilds(boolean combinebuilds) {
        this.combinebuilds = combinebuilds;
    }


    @Parameter(defaultValue = "false")
    protected boolean showBuildAndRunOutput;
    public boolean isShowBuildAndRunOutput() {
        return showBuildAndRunOutput;
    }
    public void setShowBuildAndRunOutput(boolean showBuildAndRunOutput) {
        this.showBuildAndRunOutput = showBuildAndRunOutput;
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


    @Parameter(defaultValue = "false")
    protected boolean enableExecutionLogging;
    public boolean getEnableExecutionLogging(){
        return enableExecutionLogging;
    }
    public void setEnableExecutionLogging(boolean enableExecutionLogging){
        this.enableExecutionLogging = enableExecutionLogging;
    }

    @Parameter(defaultValue = "x")
    protected char forcedTraining;
    public char getForcedTraining(){return forcedTraining;}
    public void setForcedTraining(char forcedTraining){this.forcedTraining = forcedTraining;}

    
    @Parameter(defaultValue = "false")
    protected boolean importArmadillo = false;
    public boolean getImportArdmadillo() {
        return importArmadillo;
    }
    @Parameter(defaultValue = "false")
    protected boolean generateServerAdapter = false;
    public boolean getGenerateServerAdapter() {
        return generateServerAdapter;
    }
    @Parameter(defaultValue = "false")
    protected boolean generateLibraryInterface = false;
    public boolean getGenerateLibraryInterface() {
        return generateLibraryInterface;
    }

    //</editor-fold>

    //<editor-fold desc="Properties">

    protected static String mojoDirectory = "mojo/";

    protected LogToFile myLog = null;

    //</editor-fold>

    //<editor-fold desc="Execution">
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
            myLog = LogToFile.initFile();
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

    protected Charset getCharset() {
        return Charset.forName("utf-8");
    }

    


    //</editor-fold>

    //<editor-fold desc="Build Chain">

    protected String MojoName(){
        return "StreamTestMojoBase";
    }

    protected void copyPropertiesAndParametersTo(StreamTestMojoBase stmb){
        stmb.pathMain = pathMain;
        stmb.pathTest = pathTest;
        stmb.pathTmpOut = pathTmpOut;
        stmb.wrapperTestExtension = wrapperTestExtension;
        stmb.generator = generator;
        stmb.backend = backend;
        stmb.combinebuilds = combinebuilds;
        stmb.showBuildAndRunOutput = showBuildAndRunOutput;
        stmb.forceRun = forceRun;
        stmb.showDateAndTime = showDateAndTime;
        stmb.enableExecutionLogging = enableExecutionLogging;
        stmb.trainingNeeded = trainingNeeded;
        stmb.pathToPython = pathToPython;
        stmb.rootModel = rootModel;
        stmb.customFilesPath = customFilesPath;
        stmb.importArmadillo = importArmadillo;
        stmb.generateLibraryInterface = generateLibraryInterface;
        stmb.generateServerAdapter = generateServerAdapter;
        stmb.useDgl = useDgl;

        stmb.setLog(getLog());

        stmb.myLog = myLog;


    }

    //</editor-fold>

    //<editor-fold desc="Log">



    protected void resetToMyLog(){
        if(myLog == null){
            myLog = LogToFile.initFile();
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

    public boolean isShowDateAndTime(){
        return showDateAndTime;
    }

    public void setShowDateAndTime(boolean value){
        showDateAndTime = value;
    }

    public String getCurrentLocalDateTimeStamp() {
        return LocalDateTime.now()
                .format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss.SSS"));
    }

    public String addDateTime(String msg){
        return String.format("[%s] %s", getCurrentLocalDateTimeStamp(), msg);
    }

    //</editor-fold>

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

    //<editor-fold desc="Parser, Scope, Tagging">

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

    private Scope myScope = null;
    private TaggingResolver myTaggingResolver = null;
    protected Scope getScope(){
        if(myScope == null) {
            ModelingLanguageFamily fam = new ModelingLanguageFamily();
            /*TODO: To delete*/
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());

            fam.addModelingLanguage(new StreamUnitsLanguage());
            fam.addModelingLanguage(new StructLanguage());
            fam.addModelingLanguage(new EnumLangLanguage());
            fam.addModelingLanguage(new EMADLLanguage());
            final ModelPath mp_main = new ModelPath();

            mp_main.addEntry(Paths.get(this.pathMain));
            if (!this.pathMain.equals(this.pathTest)) {
                mp_main.addEntry(Paths.get(this.pathTest));
            }

            GlobalScope gs = new GlobalScope(mp_main, fam);
            de.monticore.lang.monticar.Utils.addBuiltInTypes(gs);

            ArrayList<String> col = new ArrayList<>();

            col.add(this.pathMain);
            if (!this.pathMain.equals(this.pathTest)) {
                col.add(this.pathTest);
            }

            this.myTaggingResolver = new TaggingResolver(gs, col);
            TagMinMaxTagSchema.registerTagTypes(this.myTaggingResolver);
            TagTableTagSchema.registerTagTypes(this.myTaggingResolver);
            TagBreakpointsTagSchema.registerTagTypes(this.myTaggingResolver);
            TagExecutionOrderTagSchema.registerTagTypes(this.myTaggingResolver);
            TagInitTagSchema.registerTagTypes(this.myTaggingResolver);
            TagThresholdTagSchema.registerTagTypes(this.myTaggingResolver);
            TagDelayTagSchema.registerTagTypes(this.myTaggingResolver);
            resetToMyLog();
            myScope = gs;
        }
        return myScope;
    }
    protected TaggingResolver getTaggingResolver(){
        if(this.myTaggingResolver == null){
            this.getScope();
        }
        return this.myTaggingResolver;
    }

    protected void resetTaggingResolver(){
        this.myScope = null;
        this.myTaggingResolver = null;
    }

    //</editor-fold>

    //<editor-fold desc="Get Component Symbols">

    protected List<EMAComponentSymbol> getToTestComponentSymbols(boolean output){


        ComponentScanner componentScannerEMADL = new ComponentScanner(Paths.get(this.pathMain), this.getScope(), "emadl");
        Set<String> componentNames = componentScannerEMADL.scan();

        ComponentScanner componentScannerEMAM = new ComponentScanner(Paths.get(this.pathMain), this.getScope(), "emam");
        componentNames.addAll(componentScannerEMAM.scan());


        StreamScanner scanner = new StreamScanner(Paths.get(this.pathTest), this.getScope());
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
            if (this.trainingNeeded) {
                if (StringUtils.equals(componentName, getRootModel())) {
                    cs = getScope().resolve(componentName, EMAComponentSymbol.KIND);
                    toTestComponents.add(cs.get());
                }
            } else {
                if(cs.isPresent()) {
                    if(output){
                        logInfo("   -> Test with component: "+cs.get().getFullName()+" :");
                        streamTests.get(cs.get()).forEach( (ComponentStreamUnitsSymbol csus) -> logInfo("      # Streamtest: "+csus.getFullName()));
                    }
                    toTestComponents.add(cs.get());
                } else {
                    if(output) {
                        logWarn("   -> No streamtest found for " + componentName);
                    }
                }
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

    //</editor-fold>

    //<editor-fold desc="Process Helper">

    protected int processRun(List<String> cmd, String directory, File outputFile, File errorFile, String logPrefix)throws MojoExecutionException{
        ProcessBuilder processBuilder = new ProcessBuilder(cmd);
        processBuilder.directory(Paths.get(directory).toFile());
        try {
            processBuilder.redirectErrorStream(true);
            processBuilder.redirectError(errorFile);
            processBuilder.redirectOutput(outputFile);

            Process process = processBuilder.start();


            int result = process.waitFor();

            if(result != 0 || showBuildAndRunOutput){
                List<String> allLines = FileUtils.readLines(errorFile);
                for (String s:allLines) {
                    if(result != 0)
                        logError("   ["+logPrefix+"-errout] " + s);
                    else
                        logInfo("   ["+logPrefix+"-errout] " + s);
                }
                allLines = FileUtils.readLines(outputFile);
                for (String s:allLines) {
                    if(result != 0)
                        logError("   ["+logPrefix+"-output] " + s);
                    else
                        logInfo("   ["+logPrefix+"-output] " + s);
                }
            }

            return result;
        }catch (Exception ex){
            ex.printStackTrace();
            throw new MojoExecutionException(ex.getMessage());
        }
    }

    protected String fullNameToCMakeTarget(String name){

//        cs.getName().substring(0,1).toLowerCase()+cs.getName().substring(1);
        // name.lastIndexOf(".")
        int idx = name.lastIndexOf(".");

        name = name.substring(0, idx+1)+name.substring(idx+1,idx+2).toLowerCase()+name.substring(idx+2)+"_StreamTests";

        return name.replace(".", "_");
    }

    protected String execFileName(String name, GeneratorEnum generator){
        String targetname = fullNameToCMakeTarget(name);
        if (SystemUtils.IS_OS_WINDOWS) {
            switch (generator) {
                case VS2017:
                case VisualStudio2017:
                    return "Debug/"+targetname+".exe";
                default:
                    return targetname+".exe";
            }
        }else{
            return targetname;
        }
    }

    //</editor-fold>

    protected static String getReadableTime(Long nanos){

        long tempSec = nanos/(1000*1000*1000);
        long ms = (nanos/(1000*1000))%1000;
        long sec = tempSec % 60;
        long min = (tempSec /60) % 60;
        long hour = (tempSec /(60*60));
        return String.format("%02d:%02d:%02d.%03d",hour,min,sec,ms);

    }
}
