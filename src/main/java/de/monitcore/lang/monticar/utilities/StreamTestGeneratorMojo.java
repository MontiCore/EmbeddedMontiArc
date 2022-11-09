/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities;

import de.monitcore.lang.monticar.utilities.tools.ChecksumChecker;
import de.monitcore.lang.monticar.utilities.tools.SearchFiles;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
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
import de.rwth.montisim.commons.utils.json.Json;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.IOUtils;
import org.apache.commons.io.filefilter.WildcardFileFilter;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;
import org.json.JSONObject;

import java.io.*;
import java.nio.charset.Charset;
import java.nio.file.Path;
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
        this.generateHashFile();
    }

    @Override
    protected void mainExecution() throws MojoExecutionException {
        Log.info("StreamTestGeneratorMojo", "StreamTestGeneratorMojo");

        if(!checkCocosOfInputFiles()){
            throw new MojoExecutionException("Some files are invalid");
        }

        try{
            File temam = Paths.get(this.getPathTmpOutEMAM()).toFile();
            FileUtils.copyDirectory(Paths.get(this.pathMain).toFile(), temam);
            FileUtils.copyDirectory(Paths.get(this.pathTest).toFile(), temam);
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Could not copy files: " + e.getMessage());
        }

        List<EMAComponentSymbol> toTest = getToTestComponentSymbols(true);
        generateCPP(toTest);
    }

    @Override
    protected void postExecution() throws MojoExecutionException {
        super.postExecution();

        File oldHashFile = getHashFile().toFile();
        System.out.println(oldHashFile.getAbsolutePath());
        if(oldHashFile.exists() && !oldHashFile.delete()){
            logError("Deletion of file " + oldHashFile.getName() + " failed.");
        }

        File newHashFile = getNewHashFile().toFile();
        System.out.println(newHashFile.getAbsolutePath());

        if(!newHashFile.exists()){
            logError("File " + newHashFile.getName() + " does not exist!");
        }
        if(!newHashFile.renameTo(oldHashFile)){
            logError("Renaming file " + newHashFile.getName() + " to file " + oldHashFile.getName() + " failed.");
        };
    }

    @Override
    protected boolean checkForExecution() throws MojoExecutionException {
        JSONObject hashes;
        JSONObject newHashes;

        try {
            hashes = new JSONObject(getHashFileContent());
        } catch (FileNotFoundException e){
            logInfo("Execution necessary: Hashfile not found.");
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Exception while reading old hash files");
        }

        try {
            newHashes = new JSONObject(getHashFileContent());
        } catch (FileNotFoundException e){
            throw new MojoExecutionException("New intermediate hashfile not found, but required for execution.");
        } catch (IOException e) {
            e.printStackTrace();
            throw new MojoExecutionException("Exception while reading the intermediate new hash files");
        }

        for(String key : newHashes.keySet()){
            if(newHashes.getString(key).equalsIgnoreCase(hashes.getString(key))){
                logDebug("Input file" + key + " did not change.");
            } else {
                logInfo("Input file " + key + " did change. Execution necessary.");
                return true;
            }
        }

        logInfo("Execution not necessary: No files changed.");
        return false;
    }

    private JSONObject generateHashFile() throws MojoExecutionException {
        JSONObject newHashes = new JSONObject();
        newHashes.put("main", SearchFiles.hashDirFiles(this.getPathMain()));
        newHashes.put("test", SearchFiles.hashDirFiles(this.getPathTest()));
        newHashes.put("emam", SearchFiles.hashDirFiles(this.getPathTmpOutEMAM()));

        File newHashFile = getNewHashFile().toFile();
        if(newHashFile.exists()){
            newHashFile.delete();
        }

        try {
            FileUtils.writeStringToFile(newHashFile, newHashes.toString(), Charset.defaultCharset());
        } catch (IOException e){
            e.printStackTrace();
            throw new MojoExecutionException("Could not write to file " + newHashFile.getAbsolutePath());
        }

        return newHashes;
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
                    e.printStackTrace();
                    Log.error("io error during generation", e);
                    System.exit(1);
                }
                catch (TemplateException e){
                    e.printStackTrace();
                    Log.error("template error during generation", e);
                    System.exit(1);
                }
            }

            resetTaggingResolver();
        }
    }


    //<editor-fold desc="Hashfiles">

    private Path getHashFile(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "hashes.json");
    }

    private Path getNewHashFile(){
        return Paths.get(this.pathTmpOut, mojoDirectory, this.MojoName(), "new_hashes.json");
    }

    private String getHashFileContent() throws IOException {
        try(FileInputStream hashInputStream = new FileInputStream(getHashFile().toString())) {
            return IOUtils.toString(hashInputStream, Charset.defaultCharset());
        }
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
