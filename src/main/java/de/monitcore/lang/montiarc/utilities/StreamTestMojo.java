package de.monitcore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.tools.AllTemplates;
import de.monitcore.lang.montiarc.utilities.tools.NoLog;
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
import de.monticore.lang.monticar.enumlang._parser.EnumLangParser;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
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
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateExceptionHandler;
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
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;



@Mojo(name = "emam-streamtest")
public class StreamTestMojo extends AbstractMojo {

    private static final String EXEC_FILENAME_UNIX = "build/StreamTests";
    //private static final String EXEC_FILENAME_WINDOWS= "build/StreamTests.exe";


    private static final Template BUILD_UNIX;
    private static final Template BUILD_WINDOWS;
    private static final Template BUILD_COMBINED_CMAKELISTS;
    private static final Template BUILD_COMBINED_MAIN;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template/");
        try {
            BUILD_UNIX = conf.getTemplate("build/linux_build.ftl");
            BUILD_WINDOWS = conf.getTemplate("build/windows_build.ftl");

            BUILD_COMBINED_CMAKELISTS = conf.getTemplate("manybuild/cmakelists.ftl");
            BUILD_COMBINED_MAIN = conf.getTemplate("manybuild/main.ftl");
        } catch (IOException e) {
            String msg = "could not load cmake templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }


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

    /*@Parameter(property = "gpp", defaultValue = "g++")
    private String gpp;
    public String getGpp() {
        return gpp;
    }
    public void setGpp(String gpp) {
        this.gpp = gpp;
    }*/

    /*@Parameter
    private String[] cppInludePaths;
    public String[] getCppInludePaths(){
        return cppInludePaths;
    }
    public void setCppInludePaths(String[] paths){
        cppInludePaths = paths;
    }
    public void addCppIncludePaths(String newItem){
        int n = cppInludePaths.length;
        cppInludePaths = Arrays.copyOf(cppInludePaths, n + 1);
        cppInludePaths[n] = newItem;
    }*/

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

    /*@Parameter(name = "usemingw")
    private boolean usemingw = true;
    public boolean isUsemingw(){
        return usemingw;
    }
    public void setUsemingw(boolean usemingw) {
        this.usemingw = usemingw;
    }*/



    @Parameter(defaultValue = "NONE")
    private GeneratorEnum generator;
    public GeneratorEnum getGenerator() {
        return generator;
    }
    public void setGenerator(GeneratorEnum generator) {
        this.generator = generator;
    }




    @Parameter(defaultValue = "false")
    private boolean combinebuilds;
    public boolean getCombinebuilds() {
        return combinebuilds;
    }
    public void setCombinebuilds(boolean combinebuilds) {
        this.combinebuilds = combinebuilds;
    }


    @Parameter(defaultValue = "false")
    private boolean showBuildAndRunOutput;
    public boolean isShowBuildAndRunOutput() {
        return showBuildAndRunOutput;
    }
    public void setShowBuildAndRunOutput(boolean showBuildAndRunOutput) {
        this.showBuildAndRunOutput = showBuildAndRunOutput;
    }


    //</editor-fold>


    //<editor-fold desc="Parser, Scope and Taggingresolver">
    protected  Map<String, MCConcreteParser> parser = new HashMap<>();
    public void createParser(){
        //EmbeddedMontiArcMathParser parserEMAM = new EmbeddedMontiArcMathParser();

        this.parser.put("emam", new EmbeddedMontiArcMathParser());
        this.parser.put("stream", new StreamUnitsParser());
        this.parser.put("struct", new StructParser());
        this.parser.put("enum", new EnumLangParser());
    }

    protected GlobalScope mainScope = null;
    protected TaggingResolver tagging = null;
    public void createScopeAndTaggingResolver(){
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        fam.addModelingLanguage(new StreamUnitsLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        final ModelPath mp_main = new ModelPath();

        mp_main.addEntry(Paths.get(this.pathMain));
        if(!this.pathMain.equals(this.pathTest)) {
            mp_main.addEntry(Paths.get(this.pathTest));
        }

        this.mainScope = new GlobalScope(mp_main, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(mainScope);

        ArrayList<String> col = new ArrayList<String>();

        col.add(this.pathMain);
        if(!this.pathMain.equals(this.pathTest)) {
            col.add(this.pathTest);
        }

        this.tagging = new TaggingResolver(mainScope, col);
        TagMinMaxTagSchema.registerTagTypes(tagging);
        TagTableTagSchema.registerTagTypes(tagging);
        TagBreakpointsTagSchema.registerTagTypes(tagging);
        TagExecutionOrderTagSchema.registerTagTypes(tagging);
        TagInitTagSchema.registerTagTypes(tagging);
        TagThresholdTagSchema.registerTagTypes(tagging);
        TagDelayTagSchema.registerTagTypes(tagging);
    }

    public void init(){
        Log.enableFailQuick(false);
        getLog().debug("Create Parsers");
        this.createParser();
        this.createScopeAndTaggingResolver();

        NoLog.init();
    }

    //</editor-fold>


    protected Set<String> AllEmamComponentNames = null;
    protected Map<ComponentSymbol, Set<ComponentStreamUnitsSymbol>> AllAvailableStreamTests = null;
    protected Set<ComponentSymbol>  AllTestComponentSymbols = null;
    @Override
    public void execute() throws MojoExecutionException, MojoFailureException {

        this.init();

        getLog().debug("Cocos test of"+this.pathMain);
        Map<String, File> mainFiles = SearchFiles.searchFilesMap(this.pathMain, "emam", "struct", "enum");
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

        this.setupTmpFolder();
        this.createBuildFiles();
        this.searchTestComponentSymbols();

        this.genCppForAllTests();
        this.compileCppForAllTests();
        this.runExecTest();

        /*for (String componentName: AllEmamComponentNames) {
            ComponentSymbol cs = getTestComponentSymbol(componentName);

            if(cs == null){
                getLog().warn("No Streamtest found for "+componentName);

            }else{

                getLog().debug("Streamtest found for"+componentName+" it is "+cs.get().getFullName());
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

        }*/
    }

    public void searchTestComponentSymbols(){
        AllTestComponentSymbols = new HashSet<>();
        for (String componentName: AllEmamComponentNames) {
            ComponentSymbol cs = getTestComponentSymbol(componentName);

            if (cs == null) {
                getLog().warn("No Streamtest found for " + componentName);

            } else {
                AllTestComponentSymbols.add(cs);
            }
        }

    }

    public ComponentSymbol getTestComponentSymbol(String componentName){
        Optional<ComponentSymbol> cs = FindComponentSymbolByName(componentName, AllAvailableStreamTests.keySet());
        if(!cs.isPresent()){
            cs = FindComponentSymbolByName(componentName+this.wrapperTestExtension, AllAvailableStreamTests.keySet());
        }
        if(!cs.isPresent()){
            return null;
        }
        return cs.get();
    }

    public void genCppForAllTests() throws MojoExecutionException {
        for (ComponentSymbol cs :AllTestComponentSymbols) {
            genCppTestForComponent(cs);
        }
    }

    public List<File> genCppTestForComponent(ComponentSymbol cs) throws MojoExecutionException {
// Todo write test
        String testName = cs.getFullName();
        int idx = testName.lastIndexOf(".");
        testName = testName.substring(0,idx+1) + testName.substring(idx+1,idx+2).toLowerCase() + testName.substring(idx+2);

        ExpandedComponentInstanceSymbol ecis = mainScope.<ExpandedComponentInstanceSymbol>resolve(testName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        if(ecis == null){
            getLog().error("Error resolving "+cs.getFullName());
            throw new MojoExecutionException("Error resolving "+cs.getFullName());

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
        //use cmake
        generatorCPP.setGenerateCMake(true);
        List<File> files = null;
        try {
            files = generatorCPP.generateFiles(tagging, ecis, mainScope);
        }catch (IOException ioex){
            getLog().error("IOException generating cpp files for "+cs.getFullName());
            throw new MojoExecutionException("IOException generating cpp files for "+cs.getFullName());
        }

        return files;
    }

    public void compileCppForAllTests() throws MojoExecutionException {
        if(!this.combinebuilds) {
            for (ComponentSymbol cs : AllTestComponentSymbols) {
                compileCppTestForComponent(cs.getFullName());
            }
        }else{
            this.compileCppCombined();

        }
    }

    private boolean compileCppTestForComponent(String name) throws MojoExecutionException {
        // Todo write test
        ProcessBuilder processBuilder;
        String execfilename;
        if (SystemUtils.IS_OS_WINDOWS) {
            //processBuilder = new ProcessBuilder("../build.bat");
            processBuilder = new ProcessBuilder(Paths.get(this.getPathTmpOutCPP(), name, "../build.bat").toAbsolutePath().toString());
            execfilename = this.EXEC_FILENAME_WINDOWS();
        }else{
            processBuilder=new ProcessBuilder("/bin/bash", "../build.sh");
            execfilename = EXEC_FILENAME_UNIX;
        }

        processBuilder.directory(Paths.get(this.getPathTmpOutCPP(), name).toFile());
        try {



            getLog().debug("Compiling "+name);
            Process process = processBuilder.start();
            List<String> allLines = new ArrayList<>();
            if(showBuildAndRunOutput) {
                new BufferedReader(new InputStreamReader(process.getInputStream())).lines().forEach(System.out::println);
                new BufferedReader(new InputStreamReader(process.getErrorStream())).lines().forEach(System.out::println);
            }else {
                BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream()));
                String line;
                while (((line = in.readLine()) != null)) {
                    allLines.add(line);
                }
            }
            process.waitFor();

            //check log file
            /*File logFile = Paths.get(this.getPathTmpOutCPP(), componentSymbol.getFullName(), "build.log").toFile();

            if (logFile.length() > 0) {
                // not empty
                byte[] encoded = Files.readAllBytes(logFile.toPath());
                String log = new String(encoded, Charset.defaultCharset());
                getLog().error("Error while compiling:");
                getLog().error(log);

                throw new MojoExecutionException("Error while compiling "+componentSymbol.getFullName()+": "+log);

            }*/
            if(!Paths.get(this.getPathTmpOutCPP(), name, execfilename).toFile().exists()){
                for(String l : allLines){
                    getLog().error("Build error out: "+l);
                }
                throw new MojoFailureException("Can't compile "+name);
            }

            return true;
        }catch (Exception ex){
            ex.printStackTrace();
            throw new MojoExecutionException(ex.getMessage());

        }


    }

    private void compileCppCombined() throws MojoExecutionException {
        getLog().debug("Setting up folder for combined building");
        String foldername = ".manybuild/";
        File combinedDir = Paths.get(this.getPathTmpOutCPP(), foldername).toFile();
        if(combinedDir.exists()){
            combinedDir.delete();
        }
        combinedDir.mkdirs();



        boolean first = true;
        List<String> headerIncludes = new ArrayList<>();
        List<File> files;
        try {

            File cf = Paths.get(this.getPathTmpOutCPP(), foldername, "CMakeLists.txt").toFile();
            cf.getParentFile().mkdirs();
            FileWriter fw = new FileWriter(cf);
            fw.write(AllTemplates.generate(BUILD_COMBINED_CMAKELISTS, new HashMap<>()));
            fw.flush();
            fw.close();

            Path local = Paths.get(this.getPathTmpOutCPP(), foldername);

            for (ComponentSymbol cs : AllTestComponentSymbols) {
                if(first){

                    FileUtils.copyDirectory(Paths.get(this.getPathTmpOutCPP(), cs.getFullName(), "cmake/").toFile(),
                                            Paths.get(this.getPathTmpOutCPP(), foldername, "cmake/").toFile() );
                }
                files = SearchFiles.searchFiles(Paths.get(this.getPathTmpOutCPP(), cs.getFullName(), "test/").toFile(), "hpp");
                for (File f : files) {
                    if(!f.getName().startsWith("catch")) {
                        headerIncludes.add(local.relativize(Paths.get(f.getPath())).toString());
                    }
                }
                first = false;
            }

            cf = Paths.get(this.getPathTmpOutCPP(), foldername, "combined_tests_main.cpp").toFile();
            cf.getParentFile().mkdirs();
            fw = new FileWriter(cf);
            Map root = new HashMap();
            root.put("headerinclude", headerIncludes);
            fw.write(AllTemplates.generate(BUILD_COMBINED_MAIN, root));
            fw.flush();
            fw.close();

            compileCppTestForComponent(".manybuild");
        }catch (Exception ex){
            ex.printStackTrace();
            throw new MojoExecutionException(ex.getMessage());

        }

    }

    public void runExecTest() throws MojoExecutionException, MojoFailureException {
        boolean allTestPassed = true;
        if(!combinebuilds) {
            for (ComponentSymbol cs : AllTestComponentSymbols) {
                getLog().debug("Running main.exec  test for " + cs.getFullName());
                if (!runExecTestForComponent(cs.getFullName())) {
                    getLog().error("Tests for " + cs.getFullName() + " failed");
                    allTestPassed = false;
                }
            }
        }else{
            getLog().debug("Running main.exec test for many build");
            allTestPassed = runExecTestForComponent(".manybuild/");
            getLog().debug("Passed");
        }
        if(!allTestPassed){
            throw new MojoFailureException("Not all test passed");
        }
    }

    private boolean runExecTestForComponent(String path) throws MojoExecutionException {
        // Todo write test
        ProcessBuilder processBuilder;
        if (SystemUtils.IS_OS_WINDOWS) {
            processBuilder=new ProcessBuilder(Paths.get(this.getPathTmpOutCPP(), path, this.EXEC_FILENAME_WINDOWS()).toAbsolutePath().toString());
        }else{
            processBuilder=new ProcessBuilder("./"+EXEC_FILENAME_UNIX);
        }
        processBuilder.directory(Paths.get(this.getPathTmpOutCPP(), path).toFile());
        try{


            Process process = processBuilder.start();
            List<String> allLines = new ArrayList<>();
            if(showBuildAndRunOutput) {
                new BufferedReader(new InputStreamReader(process.getInputStream())).lines().forEach(System.out::println);
                new BufferedReader(new InputStreamReader(process.getErrorStream())).lines().forEach(System.out::println);
            }else {
                BufferedReader in = new BufferedReader(new InputStreamReader(process.getInputStream()));
                String line;
                while (((line = in.readLine()) != null)) {
                    allLines.add(line);
                }
            }
            process.waitFor();


            if(allLines.size() >= 2 && allLines.get(allLines.size()-2).contains("All tests passed")){
                return true;
            }


            for(String l : allLines){
                getLog().error(l);
            }

        }catch (Exception ex){
            //ex.printStackTrace();
            getLog().error(ex.getMessage());
            throw new MojoExecutionException(ex.getMessage());
        }
        return false;
    }

    //tested
    public void setupTmpFolder() throws MojoExecutionException {
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
        AllEmamComponentNames = componentScanner.scan();

        StreamScanner scanner = new StreamScanner(Paths.get(this.pathTest), mainScope);
        AllAvailableStreamTests = new HashMap<>(scanner.scan());
    }

    //tested
    public void createBuildFiles() throws MojoExecutionException{
        getLog().debug("Creating build files");
        try {
            Map root = new HashMap();
            root.put("user", "Big Joe");
            //root.put("cppIncludes", cppInludePaths);
            //root.put("gppCommand", this.gpp);
            root.put("genNONE", this.generator == GeneratorEnum.NONE);
            /*root.put("genMinGW", this.generator == GeneratorEnum.MinGW);
            root.put("genVS2017", ((this.generator == GeneratorEnum.VisualStudio2017) || (this.generator == GeneratorEnum.VS2017)) );
            */
            root.put("generator", this.Windows_Build_Get_GeneratorString());
            if((this.generator == GeneratorEnum.VS2017) || (this.generator == GeneratorEnum.VisualStudio2017)){
                root.put("buildoptions", "--config Debug");
            }else{
                root.put("buildoptions", "");
            }

            // BUILD.sh
            //root.put("execName", EXEC_FILENAME_UNIX);
            File f = Paths.get(this.getPathTmpOutCPP(), "build.sh").toFile();
            f.getParentFile().mkdirs();
            FileWriter fw = new FileWriter(f);
            fw.write(AllTemplates.generate(BUILD_UNIX, root));
            fw.flush();
            fw.close();


            // BUILD.bat
            //root.replace("execName", this.EXEC_FILENAME_WINDOWS());
            f = Paths.get(this.getPathTmpOutCPP(), "build.bat").toFile();
            f.getParentFile().mkdirs();
            fw = new FileWriter(f);
            fw.write(AllTemplates.generate(BUILD_WINDOWS, root));
            fw.flush();
            fw.close();
        }catch (Exception ex){
            throw new MojoExecutionException("Can't create build file for linux: "+ex.getMessage());
            //getLog().error("Can't create build file for linux: "+ex.getMessage());
        }

    }

    //tested
    public List<String> execute_ParserTests(Map<String, File> files) throws MojoExecutionException {

        List<String> errors = new ArrayList<>();
        for (Map.Entry<String,File> f:files.entrySet()) {
            String ending = f.getKey().substring(f.getKey().lastIndexOf(".")+1);
            if(!parser.keySet().contains(ending)){
                throw new MojoExecutionException("No parser for ."+ending+" files");
                //errors.add(f.getKey()+" (error: no parser for file)");
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

                        String PackageName = Joiners.DOT.join(ast.getEMACompilationUnit().getPackageList());
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
                        String PackageName = Joiners.DOT.join(ast.getPackageList());
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
                //e.printStackTrace();
                errors.add(f.getKey()+" (error: Exception while parsing. "+e.getStackTrace().toString()+")");
            }
        }
        return errors;
    }

    protected Optional<ComponentSymbol> FindComponentSymbolByName(String name, Set<ComponentSymbol> componentSymbols){
        for (ComponentSymbol cs :componentSymbols) {
            if(cs.getFullName().equals(name)){
                return Optional.of(cs);
            }
        }

        return Optional.empty();
    }

    protected String EXEC_FILENAME_WINDOWS(){
        if(this.generator == GeneratorEnum.NONE){
            return "build/StreamTests.exe";
        }
        if(this.generator == GeneratorEnum.MinGW){
            return "build/StreamTests.exe";
        }
        if((this.generator == GeneratorEnum.VS2017) || (this.generator == GeneratorEnum.VisualStudio2017)){
            return "build/Debug/StreamTests.exe";
        }
        return "";
    }

    protected String Windows_Build_Get_GeneratorString(){
        if(this.generator == GeneratorEnum.MinGW){
            return "MinGW Makefiles";
        }
        if((this.generator == GeneratorEnum.VS2017) || (this.generator == GeneratorEnum.VisualStudio2017)){
            return "Visual Studio 15 2017 Win64";
        }
        return "";
    }
}
