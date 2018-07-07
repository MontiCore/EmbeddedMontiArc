package de.monitcore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.tools.SearchFiles;
import de.monticore.ModelingLanguageFamily;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamUnitsCompilationUnit;
import de.monticore.lang.monticar.streamunits._parser.StreamUnitsParser;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import jdk.nashorn.internal.objects.annotations.Property;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.SystemUtils;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;

import java.io.*;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;

@Mojo(name = "emam-streamtest")
public class StreamTestMojo extends AbstractMojo {

    //<editor-fold desc="Parameter Definitions">

    @Parameter(property = "pathMain",defaultValue = "./src/main/emam/")
    private String pathMain;
    public String getPathMain() {
        return pathMain;
    }
    public void setPathMain(String path) {
        this.pathMain = path;
    }

    @Parameter(property = "pathTest",defaultValue = "./src/test/emam/")
    private String pathTest;
    public String getPathTest() {
        return pathTest;
    }
    public void setPathTest(String pathTest) {
        this.pathTest = pathTest;
    }

    @Parameter(property = "gpp", defaultValue = "g++")
    private String gpp;
    public String getGpp() {
        return gpp;
    }
    public void setGpp(String gpp) {
        this.gpp = gpp;
    }

    @Parameter(property = "gppPathToArmadilloH", defaultValue = "")
    private String gppPathToArmadilloH;
    public String getGppPathToArmadilloH() {
        return gppPathToArmadilloH;
    }
    public void setGppPathToArmadilloH(String gppPathToArmadilloH) {
        this.gppPathToArmadilloH = gppPathToArmadilloH;
    }

    @Parameter(property = "pathTmpOut", defaultValue = "./target/tmp")
    private String pathTmpOut;
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


    @Parameter(name = "wrapperTestExtension", defaultValue = "_TestWrapper")
    private String wrapperTestExtension;
    public String getWrapperTestExtension() {
        return wrapperTestExtension;
    }
    public void setWrapperTestExtension(String wrapperTestExtension) {
        this.wrapperTestExtension = wrapperTestExtension;
    }

    //</editor-fold>





    Map<String, MCConcreteParser> parser = new HashMap<>();
    public void createParser(){
//        EmbeddedMontiArcMathParser parserEMAM = new EmbeddedMontiArcMathParser();
        this.parser.put("emam", new EmbeddedMontiArcMathParser());
        this.parser.put("stream", new StreamUnitsParser());
    }

    GlobalScope mainScope = null;
    TaggingResolver tagging = null;
    public void createScopeAndTaggingResolver(){
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        fam.addModelingLanguage(new StreamUnitsLanguage());

        final ModelPath mp_main = new ModelPath();

        mp_main.addEntry(Paths.get(this.pathMain));
        mp_main.addEntry(Paths.get(this.pathTest));
        this.mainScope = new GlobalScope(mp_main, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(mainScope);

        ArrayList<String> col = new ArrayList<String>();
        col.add(this.pathMain);
        col.add(this.pathTest);
        this.tagging = new TaggingResolver(mainScope, col);
        TagMinMaxTagSchema.registerTagTypes(tagging);
        TagTableTagSchema.registerTagTypes(tagging);
        TagBreakpointsTagSchema.registerTagTypes(tagging);
        TagExecutionOrderTagSchema.registerTagTypes(tagging);
        TagInitTagSchema.registerTagTypes(tagging);
        TagThresholdTagSchema.registerTagTypes(tagging);
        TagDelayTagSchema.registerTagTypes(tagging);
    }

    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {
        Log.enableFailQuick(false);

        getLog().debug("Create Parsers");
        this.createParser();
        this.createScopeAndTaggingResolver();

        getLog().debug("Cocos test of"+this.pathMain);
        Map<String, File> mainFiles = SearchFiles.searchFilesMap(this.pathMain, "emam", "stream");
        List<String> errorFiles = this.execute_ParserTests(mainFiles);

        getLog().debug("Cocos test of"+this.pathTest);
        Map<String, File> testFiles = SearchFiles.searchFilesMap(this.pathTest, "emam", "stream");
        errorFiles.addAll(this.execute_ParserTests(testFiles));

        if(!errorFiles.isEmpty()){
            getLog().error("Error in file(s)");
            for (String e: errorFiles) {
                getLog().error(e);
            }
            throw new MojoFailureException("Error in file(s): "+String.join(", ", errorFiles));
        }

        getLog().debug("Setting up tmp folders");

        File mainDir = new File(this.pathMain);
        File testsDir = new File(this.pathTest);
        File temam = new File(this.getPathTmpOutEMAM());
        File tcpp = new File(this.getPathTmpOutCPP());
        try {
            FileUtils.deleteDirectory(temam);
            FileUtils.deleteDirectory(tcpp);

            FileUtils.copyDirectory(mainDir, temam);
            FileUtils.copyDirectory(testsDir, temam);
        } catch (Exception e) {
            e.printStackTrace();
            throw new MojoExecutionException("Error creating tmp folders", e);
        }

        getLog().debug("Searching components and tests");
        ComponentScanner componentScanner = new ComponentScanner(Paths.get(this.pathMain), mainScope, "emam");
        Set<String> emamComponentNames = componentScanner.scan();

        StreamScanner scanner = new StreamScanner(Paths.get(this.pathTest), mainScope);
        Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreamTests = new HashMap<>(scanner.scan());

        getLog().debug("Creating build files");
        this.createBuildFiles();

        for (String componentName: emamComponentNames) {
            Optional<ComponentSymbol> cs = FindComponentSymbolByName(componentName, availableStreamTests.keySet());
            if(!cs.isPresent()){
                cs = FindComponentSymbolByName(componentName+this.wrapperTestExtension, availableStreamTests.keySet());
            }

            if(!cs.isPresent()){
                getLog().warn("No Streamtest found for "+componentName);

            }else{

                getLog().debug("Streamtest found for"+componentName+" it is "+cs);
                if(this.execute_GenCppTest(cs.get())){
                    if(this.execute_CompileCppTest(cs.get()))
                    {
                        if(this.execute_RunExecTest(cs.get()))
                        {
                            getLog().info("Tests for "+cs.get().getFullName()+" success");
                        }
                    }
                }
            }

        }
    }

    private boolean execute_CompileCppTest(ComponentSymbol componentSymbol) {
        ProcessBuilder processBuilder;
        if (SystemUtils.IS_OS_WINDOWS) {
            processBuilder = null;
        }else{
            processBuilder=new ProcessBuilder("/bin/bash", "../build.sh");
            processBuilder.directory(Paths.get(this.getPathTmpOutCPP(), componentSymbol.getFullName()).toFile());

        }

        try {
            getLog().debug("Compiling "+componentSymbol.getFullName());
            Process process = processBuilder.start();
            process.waitFor();

            //check log file
            File logFile = Paths.get(this.getPathTmpOutCPP(), componentSymbol.getFullName(), "build.log").toFile();

            if (logFile.length() > 0) {
                // not empty
                byte[] encoded = Files.readAllBytes(logFile.toPath());
                String log = new String(encoded, Charset.defaultCharset());
                getLog().error("Error while compiling:");
                getLog().error(log);
                return false;
            }

            return true;
        }catch (Exception ex){
            ex.printStackTrace();
        }

        return false;
    }

    private boolean execute_RunExecTest(ComponentSymbol componentSymbol){
        ProcessBuilder processBuilder;
        if (SystemUtils.IS_OS_WINDOWS) {
            processBuilder = null;
        }else{
            processBuilder=new ProcessBuilder("./main.exec");
            processBuilder.directory(Paths.get(this.getPathTmpOutCPP(), componentSymbol.getFullName()).toFile());

        }

        try{
            getLog().debug("Running main.exec  test for "+componentSymbol.getFullName());

            Process process = processBuilder.start();
            BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream()));
            String line;
            List<String> allLines = new ArrayList<>();
            while ((line = in.readLine()) != null) {
                allLines.add(line);
                getLog().debug(line);
            }
            process.waitFor();


            if(allLines.size() >= 2 && allLines.get(allLines.size()-2).startsWith("All tests passed")){
                return true;
            }

        }catch (Exception ex){
            ex.printStackTrace();
            getLog().error(ex.getMessage());

        }
        return false;
    }

    public void createBuildFiles(){

        try {
            File f = Paths.get(this.getPathTmpOutCPP(), "build.sh").toFile();
            f.getParentFile().mkdirs();
            PrintWriter out = new PrintWriter(f);
            out.println("#!/bin/sh");
            out.println();
            //out.println(String.format("%s -std=c++11 -DCATCH_CONFIG_MAIN=1 -I\"%s\" \"test/tests_main.cpp\" -o main.exec -DARMA_DONT_USE_WRAPPER", this.gcccommand, this.gccpathtoarmadilloh));
            out.print(this.gpp);
            out.print(" -std=c++11 -DCATCH_CONFIG_MAIN=1 -DARMA_DONT_USE_WRAPPER -I\"");
            out.print(this.gppPathToArmadilloH);
            out.print("\" \"test/tests_main.cpp\" -o main.exec > build.log");
            out.println();out.println();
            out.flush();
            out.close();
        }catch (Exception ex){
            getLog().error("Can't create build file for linux: "+ex.getMessage());
        }


        // TODO : build bat for windows
        getLog().error("No implementation for windows right now");

    }

    protected List<String> execute_ParserTests(Map<String, File> files){

        List<String> errors = new ArrayList<>();
        for (Map.Entry<String,File> f:files.entrySet()) {
            String ending = f.getKey().substring(f.getKey().lastIndexOf(".")+1);
            if(!parser.keySet().contains(ending)){
                errors.add(f.getKey()+" (error: no parser for file)");
            }
            MCConcreteParser concreteParser = parser.get(ending);
            getLog().debug("Parser Test "+f.getKey());

            Optional<? extends ASTNode> astemamCompilationUnit = Optional.empty();
            try {
                astemamCompilationUnit = concreteParser.parse(f.getValue().getCanonicalPath());
                if(!astemamCompilationUnit.isPresent()){
                    errors.add(f.getKey());
                }else {

                    if(ending.equalsIgnoreCase("emam")) {
                        ASTEMAMCompilationUnit ast = (ASTEMAMCompilationUnit) astemamCompilationUnit.get();

                        String PackageName = Joiners.DOT.join(ast.getEMACompilationUnit().getPackage());
                        String modelName = PackageName + "." + f.getValue().getName().replace(".emam", "");

                        ComponentSymbol comp = mainScope.<ComponentSymbol>resolve(modelName, ComponentSymbol.KIND).orElse(null);
                        if(comp == null){
                            errors.add(f.getKey());
                        }else{
                            //TODO: add component symbol to a list
                        }
                        //ASTEmbeddedMontiArcNode resolvedAST = (ASTEmbeddedMontiArcNode) comp.getAstNode().get();
                    }else if(ending.equalsIgnoreCase("stream")){

                        ASTStreamUnitsCompilationUnit ast = (ASTStreamUnitsCompilationUnit)astemamCompilationUnit.get();
                        String PackageName = Joiners.DOT.join(ast.getPackage());
                        String modelName = PackageName + "." + f.getValue().getName().replace(".stream", "");

                        ComponentStreamUnitsSymbol comp = mainScope.<ComponentStreamUnitsSymbol>resolve(modelName, ComponentStreamUnitsSymbol.KIND).orElse(null);
                        if(comp == null){
                            errors.add(f.getKey());
                        }else{
                            //TODO: add component symbol to a list
                        }
                    }

                }

            } catch (Exception e) {
//                e.printStackTrace();
                errors.add(f.getKey()+" (error: Exception while parsing. "+e.getStackTrace().toString()+")");
            }
        }
        return errors;
    }

    protected boolean execute_GenCppTest(ComponentSymbol cs){

        String testName = cs.getFullName();
        int idx = testName.lastIndexOf(".");
        testName = testName.substring(0,idx+1) + testName.substring(idx+1,idx+2).toLowerCase() + testName.substring(idx+2);

        ExpandedComponentInstanceSymbol ecis = mainScope.<ExpandedComponentInstanceSymbol>resolve(testName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        if(ecis == null){
            getLog().error("Error resolving "+cs.getFullName());
            return false;
        }

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get(this.getPathTmpOutEMAM()));


        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath(Paths.get(this.getPathTmpOutCPP(),cs.getFullName()).toString());
        generatorCPP.setGenerateMainClass(true);
        generatorCPP.setGenerateTests(true);
        generatorCPP.setCheckModelDir(true);
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setUseThreadingOptimization(false);
        try {
            List<File> files = generatorCPP.generateFiles(tagging, ecis, mainScope);
        }catch (IOException ioex){
            getLog().error("IOException generating cpp files for "+cs.getFullName());
            return false;
        }

        return true;
    }

    protected Optional<ComponentSymbol> FindComponentSymbolByName(String name, Set<ComponentSymbol> componentSymbols){
        for (ComponentSymbol cs :componentSymbols) {
            if(cs.getFullName().equals(name)){
                return Optional.of(cs);
            }
        }

        return Optional.empty();
    }
}
