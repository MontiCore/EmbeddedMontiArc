package de.monitcore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.tools.AllTemplates;
import de.monitcore.lang.montiarc.utilities.tools.LogToFile;
import de.monticore.ModelingLanguageFamily;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
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
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Paths;
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
    public String getPathTmpOutBUILD() {
        return Paths.get(this.getPathTmpOut(), "build/").toString();
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

    //</editor-fold>

    //<editor-fold desc="Properties">

    protected static String mojoDirectory = "mojo/";

    protected LogToFile myLog = null;

    //</editor-fold>

    //<editor-fold desc="Execution">
    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        this.preExecution();
        if(checkForExecution()) {
            this.mainExecution();
            this.postExecution();
        }
    }

    protected void preExecution() throws MojoExecutionException, MojoFailureException {
        this.mkdir(this.getPathTmpOut());
        this.mkdir(this.getPathTmpOutCPP());
        this.mkdir(this.getPathTmpOutEMAM());
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

    protected void postExecution()throws MojoExecutionException{
        // hier koennte ihre werbung stehen
    }

    protected boolean checkForExecution() throws MojoExecutionException {
        return true;
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
        stmb.combinebuilds = combinebuilds;
        stmb.showBuildAndRunOutput = showBuildAndRunOutput;
        stmb.forceRun = forceRun;

        stmb.setLog(getLog());

        stmb.myLog = myLog;


    }

    //</editor-fold>

    //<editor-fold desc="Log">

    protected void resetToMyLog(){
        if(myLog == null){
            myLog = LogToFile.init();
        }else{
            LogToFile.resetTo(myLog);
        }
    }

    protected void logError(String msg){
        getLog().error(msg);
        Log.error(msg);
    }

    protected void logInfo(String msg){
        getLog().info(msg);
        Log.info(msg, this.MojoName());
    }

    protected void logWarn(String msg){
        getLog().warn(msg);
        Log.warn(msg);
    }

    protected void logDebug(String msg){
        getLog().debug(msg);
        Log.debug(msg, this.MojoName());
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
            resetToMyLog();
        }
        return myParser;
    }

    private Scope myScope = null;
    private TaggingResolver myTaggingResolver = null;
    protected Scope getScope(){
        if(myScope == null) {
            ModelingLanguageFamily fam = new ModelingLanguageFamily();
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
            fam.addModelingLanguage(new StreamUnitsLanguage());
            fam.addModelingLanguage(new StructLanguage());
            fam.addModelingLanguage(new EnumLangLanguage());
            final ModelPath mp_main = new ModelPath();

            mp_main.addEntry(Paths.get(this.pathMain));
            if (!this.pathMain.equals(this.pathTest)) {
                mp_main.addEntry(Paths.get(this.pathTest));
            }

            GlobalScope gs = new GlobalScope(mp_main, fam);
            de.monticore.lang.monticar.Utils.addBuiltInTypes(gs);

            ArrayList<String> col = new ArrayList<String>();

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

    //</editor-fold>

    //<editor-fold desc="Get Component Symbols">

    protected List<ComponentSymbol> getToTestComponentSymbols(boolean output){

        ComponentScanner componentScanner = new ComponentScanner(Paths.get(this.pathMain), this.getScope(), "emam");
        Set<String> componentNames = componentScanner.scan();

        StreamScanner scanner = new StreamScanner(Paths.get(this.pathTest), this.getScope());
        Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> streamTests = scanner.scan();

        if(output) {
            logInfo("Searching components (with wrapper) with streamtests.");
        }

        List<ComponentSymbol> toTestComponents = new ArrayList<>();
        for (String componentName: componentNames) {
            if(output){
                logInfo(" - "+componentName);
            }
            Optional<ComponentSymbol> cs = getTestComponentSymbol(componentName, streamTests.keySet());
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

    protected Optional<ComponentSymbol> getTestComponentSymbol(String componentName, Set<ComponentSymbol> componentSymbols){

        Optional<ComponentSymbol> cs = FindComponentSymbolByName(componentName, componentSymbols);
        if(!cs.isPresent()){
            cs = FindComponentSymbolByName(componentName+this.wrapperTestExtension, componentSymbols);
        }
        return cs;
    }

    protected Optional<ComponentSymbol> FindComponentSymbolByName(String name, Set<ComponentSymbol> componentSymbols){
        for (ComponentSymbol cs :componentSymbols) {
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

    protected String execFileName(GeneratorEnum generator){
        if (SystemUtils.IS_OS_WINDOWS) {
            switch (generator) {
                case VS2017:
                case VisualStudio2017:
                    return "Debug/StreamTests.exe";
                default:
                    return "StreamTests.exe";
            }
        }else{
            return "StreamTests";
        }
    }

    //</editor-fold>
}
