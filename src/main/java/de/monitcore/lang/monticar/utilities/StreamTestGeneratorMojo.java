/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;

import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.enumlang._ast.ASTEnumLangCompilationUnit;
import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamUnitsCompilationUnit;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

/**
 * Runs CoCos test on all components in pathMain and pathTest
 * Generates c++ code for all components which have a stream test
 */
@Mojo(name = "streamtest-generator")
public class StreamTestGeneratorMojo extends StreamTestMojoBase {


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
        if(!checkCocosOfInputFiles()){
            throw new MojoExecutionException("Some files are invalid");
        }

        try{
            File temam = Paths.get(this.getPathTmpOutEMAM()).toFile();
            FileUtils.copyDirectory(Paths.get(this.pathMain).toFile(), temam);
            FileUtils.copyDirectory(Paths.get(this.pathTest).toFile(), temam);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Could not copy files: "+e.getMessage() );
        }

        List<EMAComponentSymbol> toTest = getToTestComponentSymbols(true);
        generateCPP(toTest);
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


    @Override
    protected String MojoName(){
        return "streamtest-generator";
    }

    protected boolean checkCocosOfInputFiles() throws MojoExecutionException {
        boolean result = true;

        Map<String,MCConcreteParser> parser = getParser();
        Scope scope = getScope();

        logInfo("Cocos Check:");

        List<File> ff = SearchFiles.searchFiles(this.pathMain, "emam", "struct", "enum","emadl");
        Map<String, File> files = SearchFiles.searchFilesMap(this.pathMain, "emam", "struct", "enum","emadl");
        files.putAll(SearchFiles.searchFilesMap(this.pathTest, "emam", "stream","emadl"));

        for (Map.Entry<String,File> f:files.entrySet()) {
            String ending = f.getKey().substring(f.getKey().lastIndexOf(".") + 1);
            if (!parser.keySet().contains(ending)) {
                throw new MojoExecutionException("No parser for ." + ending + " files");
                //errors.add(f.getKey()+" (error: no parser for file)");
            }
            MCConcreteParser mccp = parser.get(ending);
            logInfo(" - "+f.getKey());
            try {
                Optional<? extends ASTNode> node = mccp.parse(f.getValue().getAbsolutePath());
                if(!node.isPresent()){
                    logError("   -> Could not parse: "+f.getKey());
                    result = false;
                }else {
                    boolean resolved = false;
                    String modelName;
                    if(ending.equalsIgnoreCase("emam")) {
                        ASTEMACompilationUnit ast = (ASTEMACompilationUnit) node.get();

                        modelName = modelNameCalculator(f.getValue(),"emam", ast.getPackageList());
                        Optional<EMAComponentSymbol> comp = scope.<EMAComponentSymbol>resolve(modelName, EMAComponentSymbol.KIND);
                        resolved = comp.isPresent();
                    }else if(ending.equalsIgnoreCase("stream")) {
                        ASTStreamUnitsCompilationUnit ast = (ASTStreamUnitsCompilationUnit) node.get();

                        modelName = modelNameCalculator(f.getValue(), "stream", ast.getPackageList());
                        Optional<ComponentStreamUnitsSymbol> comp = scope.<ComponentStreamUnitsSymbol>resolve(modelName, ComponentStreamUnitsSymbol.KIND);
                        resolved = comp.isPresent();
                    }else if(ending.equalsIgnoreCase("struct")) {
                        ASTStructCompilationUnit ast = (ASTStructCompilationUnit) node.get();
                        modelName = modelNameCalculator(f.getValue(), "struct", ast.getPackageList());
                        Optional<StructSymbol> structSym = scope.resolve(modelName, StructSymbol.KIND);
                        resolved = structSym.isPresent();
                    }else if(ending.equalsIgnoreCase("enum")){
                        ASTEnumLangCompilationUnit ast = (ASTEnumLangCompilationUnit)node.get();
                        modelName = modelNameCalculator(f.getValue(), "enum", ast.getPackageList());
                        Optional<EnumDeclarationSymbol> enumSym = scope.resolve(modelName, EnumDeclarationSymbol.KIND);
                        resolved = enumSym.isPresent();
                    }else if (ending.equalsIgnoreCase("emadl")){
                        ASTEMACompilationUnit ast = (ASTEMACompilationUnit) node.get();

                        modelName = modelNameCalculator(f.getValue(),"emadl", ast.getPackageList());
                        Optional<EMAComponentSymbol> comp = scope.<EMAComponentSymbol>resolve(modelName, EMAComponentSymbol.KIND);
                        resolved = comp.isPresent();
                    }else{
                        //TODO:
                        logWarn("   -> No resolving for "+ending+" implemented at the moment.");
                    }

                    if(resolved){
                        logInfo("   -> parsed & resolved");
                    }else{
                        logError("Could not resolve "+f.getKey());
                        result = false;
                    }

                }
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

        return result;

    }

    protected void generateCPP(List<EMAComponentSymbol> toTest){

        logInfo("Generate CPP:");
        for (EMAComponentSymbol cs :toTest) {
            String name = cs.getPackageName() + "." + cs.getName().substring(0,1).toLowerCase()+cs.getName().substring(1);
            logInfo(" - "+cs.getFullName()+" = "+name);

            Scope scope = getScope();
            TaggingResolver tagging = this.getTaggingResolver();
            Optional<EMAComponentInstanceSymbol> ecis = scope.<EMAComponentInstanceSymbol>resolve(name, EMAComponentInstanceSymbol.KIND);

            if(!ecis.isPresent()){
                logError("   -> Can't resolve ExpandedComponentInstanceSymbol for "+cs.getFullName());
                continue;
            }

            EMADLGenerator emadlGenerator = new EMADLGenerator(this.backend);

            if(this.getCustomFilesPath() != null) {
                emadlGenerator.setCustomFilesPath(this.getCustomFilesPath());
            }

            if(this.getUseDgl() != null) {
                emadlGenerator.setUseDgl(this.getUseDgl().equals("y"));
            }

            if(this.getPathToPython() != null) {
                emadlGenerator.setPythonPath(this.getPathToPython());
            }

            GeneratorCPP generatorCPP = emadlGenerator.getEmamGen();
            //GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.setModelsDirPath(Paths.get(this.getPathTmpOutEMAM()));

            generatorCPP.setImportArmadillo(getImportArdmadillo());
            generatorCPP.setGenerateServerAdapter(getGenerateServerAdapter());
            generatorCPP.setGenerateLibraryInterface(getGenerateLibraryInterface());
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(Paths.get(this.getPathTmpOutCPP(),cs.getFullName()).toString());
            generatorCPP.setGenerateMainClass(true);
            generatorCPP.setGenerateTests(true);
            generatorCPP.setCheckModelDir(true);
            generatorCPP.setUseAlgebraicOptimizations(false);
            generatorCPP.setUseThreadingOptimization(false);
            if(getEnableExecutionLogging()){
                logWarn("Execution logging is activated! Resulting log files get large very fast!");
            }
            generatorCPP.setExecutionLoggingActive(getEnableExecutionLogging());
            //use cmake
            generatorCPP.setGenerateCMake(true);
            List<File> files = null;
            try {
                files = generatorCPP.generateFiles(tagging, ecis.get());
                logInfo("   -> Success");



                //String s = files.stream().map(f -> f.getName()).collect(Collectors.joining(", "));
                //logInfo("   -> Files: "+s);
            }catch (IOException ioex){
                logError("   -> IOException generating cpp files for "+cs.getFullName());
            }
            // Needed, as the C++ generator modifies the Symbol Table in destructive ways


            if (this.trainingNeeded) {
                String outputPath = Paths.get(this.getPathTmpOutCPP(),cs.getFullName()).toString();
                if (outputPath != null){
                    emadlGenerator.setGenerationTargetPath(outputPath);
                }
                try{
                    emadlGenerator.generate(this.getPathMain(), this.getRootModel(), this.getPathToPython(), "x", true, this.getUseDgl());
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

            resetTaggingResolver();
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
