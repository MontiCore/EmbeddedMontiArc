/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl.generator;

import com.google.common.base.Charsets;
import com.google.common.base.Joiner;
import com.google.common.base.Splitter;
import com.google.common.io.Resources;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.cnnarch.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.DataPathConfigParser;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain.CNNTrainGenerator;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.emadl._cocos.EMADLCocos;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.ArmadilloHelper;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.monticar.generator.cpp.SimulatorIntegrationHelper;
import de.monticore.lang.monticar.generator.cpp.TypesGeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;

import java.io.*;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.DigestInputStream;

import javax.xml.bind.DatatypeConverter;

public class EMADLGenerator {

    private GeneratorEMAMOpt2CPP emamGen;
    private CNNArchGenerator cnnArchGenerator;
    private CNNTrainGenerator cnnTrainGenerator;
    private Backend backend;

    private String modelsPath;
    


    public EMADLGenerator(Backend backend) {
        this.backend = backend;
        emamGen = new GeneratorEMAMOpt2CPP();
        emamGen.useArmadilloBackend();
        emamGen.setGenerationTargetPath("./target/generated-sources-emadl/");
        cnnArchGenerator = backend.getCNNArchGenerator();
        cnnTrainGenerator = backend.getCNNTrainGenerator();
    }

    public String getModelsPath() {
        return modelsPath;
    }

    public void setModelsPath(String modelsPath) {
        if (!(modelsPath.substring(modelsPath.length() - 1).equals("/"))){
            this.modelsPath = modelsPath + "/";
        }
        else {
            this.modelsPath = modelsPath;
        }
    }

    public void setGenerationTargetPath(String generationTargetPath){
        if (!(generationTargetPath.substring(generationTargetPath.length() - 1).equals("/"))){
            getEmamGen().setGenerationTargetPath(generationTargetPath + "/");
        }
        else {
            getEmamGen().setGenerationTargetPath(generationTargetPath);
        }
    }

    public String getGenerationTargetPath(){
        return getEmamGen().getGenerationTargetPath();
    }

    public GeneratorEMAMOpt2CPP getEmamGen() {
        return emamGen;
    }

    public void generate(String modelPath, String qualifiedName, String pythonPath, String forced, boolean doCompile) throws IOException, TemplateException {
        setModelsPath( modelPath );
        TaggingResolver symtab = EMADLAbstractSymtab.createSymTabAndTaggingResolver(getModelsPath());
        EMAComponentSymbol component = symtab.<EMAComponentSymbol>resolve(qualifiedName, EMAComponentSymbol.KIND).orElse(null);

        List<String> splitName = Splitters.DOT.splitToList(qualifiedName);
        String componentName = splitName.get(splitName.size() - 1);
        String instanceName = componentName.substring(0, 1).toLowerCase() + componentName.substring(1);

        if (component == null){
            Log.error("Component with name '" + componentName + "' does not exist.");
            System.exit(1);
        }

        EMAComponentInstanceSymbol instance = component.getEnclosingScope().<EMAComponentInstanceSymbol>resolve(instanceName, EMAComponentInstanceSymbol.KIND).get();


        generateFiles(symtab, instance, symtab, pythonPath, forced);
        
        if (doCompile) {
            compile();
        }
    }

    public void compile() throws IOException {
        File tempScript = createTempScript();
        try {
            ProcessBuilder pb = new ProcessBuilder("bash", tempScript.toString());
            pb.inheritIO();
            Process process = pb.start();
            int returnCode = process.waitFor();
            if(returnCode != 0) {
                Log.error("During compilation, an error occured. See above for more details.");
                System.exit(1);
            }
        }catch(Exception e){
            Log.error("During compilation, the following error occured: '" + e.toString() + "'");
            System.exit(1);
        } finally {
            tempScript.delete();
        }
    }

    public File createTempScript() throws IOException{
        File tempScript = File.createTempFile("script", null);
        try{
            Writer streamWriter = new OutputStreamWriter(new FileOutputStream(
                tempScript));
            PrintWriter printWriter = new PrintWriter(streamWriter);

            printWriter.println("#!/bin/bash");
            printWriter.println("cd " + getGenerationTargetPath());
            printWriter.println("mkdir -p build");
            printWriter.println("cd build");
            printWriter.println("cmake ..");
            printWriter.println("make");

            printWriter.close();
        }catch(Exception e){
            System.out.println(e);
        }

        return tempScript;
    }

    public String getChecksumForFile(String filePath) throws IOException {
        Path wiki_path = Paths.get(filePath);
        MessageDigest md5;

        try {
            md5 = MessageDigest.getInstance("MD5");
            md5.update(Files.readAllBytes(wiki_path));
            byte[] digest = md5.digest();

            return DatatypeConverter.printHexBinary(digest).toUpperCase();
        } catch (NoSuchAlgorithmException e) {
            e.printStackTrace();
            return "No_Such_Algorithm_Exception";
        }
    }

    public void generateFiles(TaggingResolver taggingResolver, EMAComponentInstanceSymbol EMAComponentSymbol, Scope symtab, String pythonPath, String forced) throws IOException {
        Set<EMAComponentInstanceSymbol> allInstances = new HashSet<>();
        List<FileContent> fileContents = generateStrings(taggingResolver, EMAComponentSymbol, symtab, allInstances, forced);

        for (FileContent fileContent : fileContents) {
            emamGen.generateFile(fileContent);
        }

        // train
        Map<String, String> fileContentMap = new HashMap<>();
        for(FileContent f : fileContents) {
            fileContentMap.put(f.getFileName(), f.getFileContent());
        }

        List<FileContent> fileContentsTrainingHashes = new ArrayList<>();
        List<String> newHashes = new ArrayList<>();
        for (EMAComponentInstanceSymbol componentInstance : allInstances) {
            Optional<ArchitectureSymbol> architecture = componentInstance.getSpannedScope().resolve("", ArchitectureSymbol.KIND);

            if(!architecture.isPresent()) {
                continue;
            }

            if(forced.equals("n")) {
                continue;
            }

            String configFilename = getConfigFilename(componentInstance.getComponentType().getFullName(), componentInstance.getFullName(), componentInstance.getName());
            String emadlPath = getModelsPath() + configFilename + ".emadl";
            String cnntPath = getModelsPath() + configFilename + ".cnnt";

            String emadlHash = getChecksumForFile(emadlPath);
            String cnntHash = getChecksumForFile(cnntPath);

            String componentConfigFilename = componentInstance.getComponentType().getReferencedSymbol().getFullName().replaceAll("\\.", "/");

            String b = backend.getBackendString(backend);
            String trainingDataHash = "";
            String testDataHash = "";
            if(b.equals("CAFFE2")){
                trainingDataHash = getChecksumForFile(architecture.get().getDataPath() + "/train_lmdb/data.mdb");
                testDataHash = getChecksumForFile(architecture.get().getDataPath() + "/test_lmdb/data.mdb");
            }else{
                trainingDataHash = getChecksumForFile(architecture.get().getDataPath() + "/train.h5");
                testDataHash = getChecksumForFile(architecture.get().getDataPath() + "/test.h5");
            }
            String trainingHash = emadlHash + "#" + cnntHash + "#" + trainingDataHash + "#" + testDataHash;

            boolean alreadyTrained = newHashes.contains(trainingHash) || isAlreadyTrained(trainingHash, componentInstance);
            if(alreadyTrained && !forced.equals("y")) {
                Log.warn("Training of model " + componentInstance.getFullName() + " skipped");
            }
            else {
                String parsedFullName = componentInstance.getFullName().substring(0, 1).toLowerCase() + componentInstance.getFullName().substring(1).replaceAll("\\.", "_");
                String trainerScriptName = "CNNTrainer_" + parsedFullName + ".py";
                String trainingPath = getGenerationTargetPath() + trainerScriptName;
                if(Files.exists(Paths.get(trainingPath))){
                    ProcessBuilder pb = new ProcessBuilder(Arrays.asList(pythonPath, trainingPath)).inheritIO();
                    Process p = pb.start();

                    int exitCode = 0;
                    try {
                        exitCode = p.waitFor();
                    }
                    catch(InterruptedException e) {
                        Log.error("Training aborted: exit code " + Integer.toString(exitCode));
                        System.exit(1);
                    }

                    if(exitCode != 0) {
                        Log.error("Training failed: exit code " + Integer.toString(exitCode));
                        System.exit(1);
                    }

                    fileContentsTrainingHashes.add(new FileContent(trainingHash, componentConfigFilename + ".training_hash"));
                    newHashes.add(trainingHash);
                }
                else{
                    System.out.println("Trainingfile " + trainingPath + " not found.");
                }
            }

        }

        for (FileContent fileContent : fileContentsTrainingHashes) {
            emamGen.generateFile(fileContent);
        }
    }

    private static String convertByteArrayToHexString(byte[] arrayBytes) {
        StringBuffer stringBuffer = new StringBuffer();
        for (int i = 0; i < arrayBytes.length; i++) {
            stringBuffer.append(Integer.toString((arrayBytes[i] & 0xff) + 0x100, 16)
                    .substring(1));
        }
        return stringBuffer.toString();
    }

    private boolean isAlreadyTrained(String trainingHash, EMAComponentInstanceSymbol componentInstance) {
        try {
            EMAComponentSymbol component = componentInstance.getComponentType().getReferencedSymbol();
            String componentConfigFilename = component.getFullName().replaceAll("\\.", "/");

            String checkFilePathString = getGenerationTargetPath() + componentConfigFilename + ".training_hash";
            Path checkFilePath = Paths.get( checkFilePathString);
            if(Files.exists(checkFilePath)) {
                List<String> hashes = Files.readAllLines(checkFilePath);
                for(String hash : hashes) {
                    if(hash.equals(trainingHash)) {
                        return true;
                    }
                }
            }

            return false;
        }
        catch(Exception e) {
            return false;
        }
    }

    public List<FileContent> generateStrings(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol, Scope symtab, Set<EMAComponentInstanceSymbol> allInstances, String forced){
        List<FileContent> fileContents = new ArrayList<>();

        generateComponent(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);

        String instanceName = componentInstanceSymbol.getComponentType().getFullName().replaceAll("\\.", "_");
        fileContents.addAll(generateCNNTrainer(allInstances, instanceName));

        fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent());
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        fileContents.addAll(tg.generateTypes(TypeConverter.getTypeSymbols()));

        if (cnnArchGenerator.isCMakeRequired()) {
            cnnArchGenerator.setGenerationTargetPath(getGenerationTargetPath());
            Map<String, String> cmakeContentsMap = cnnArchGenerator.generateCMakeContent(componentInstanceSymbol.getFullName());
            for (String fileName : cmakeContentsMap.keySet()){
                fileContents.add(new FileContent(cmakeContentsMap.get(fileName), fileName));
            }
        }

        if (emamGen.shouldGenerateMainClass()) {
            //fileContents.add(emamGen.getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (emamGen.shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        fixArmadilloImports(fileContents);

        return fileContents;
    }

    protected String getDataPath(TaggingResolver taggingResolver, EMAComponentSymbol component, EMAComponentInstanceSymbol instance){
        // instance tags have priority
        List<TagSymbol> instanceTags = (List<TagSymbol>) taggingResolver.getTags(instance, DataPathSymbol.KIND);
        List<TagSymbol> tags = !instanceTags.isEmpty() ? instanceTags :
                (List<TagSymbol>) taggingResolver.getTags(component, DataPathSymbol.KIND);

        String dataPath;
        if (!tags.isEmpty()) {
            dataPath = (String) tags.get(0).getValues().get(0);
        }
        else {
            DataPathConfigParser newParserConfig = new DataPathConfigParser(getModelsPath() + "data_paths.txt");
            dataPath = newParserConfig.getDataPath(component.getFullName());
        }

        return dataPath;
    }

    protected void generateComponent(List<FileContent> fileContents,
                                     Set<EMAComponentInstanceSymbol> allInstances,
                                     TaggingResolver taggingResolver,
                                     EMAComponentInstanceSymbol componentInstanceSymbol,
                                     Scope symtab){
        allInstances.add(componentInstanceSymbol);
        EMAComponentSymbol EMAComponentSymbol = componentInstanceSymbol.getComponentType().getReferencedSymbol();

        /* remove the following two lines if the component symbol full name bug with generic variables is fixed */
        EMAComponentSymbol.setFullName(null);
        EMAComponentSymbol.getFullName();
        /* */

        Optional<ArchitectureSymbol> architecture = componentInstanceSymbol.getSpannedScope().resolve("", ArchitectureSymbol.KIND);
        Optional<MathStatementsSymbol> mathStatements = EMAComponentSymbol.getSpannedScope().resolve("MathStatements", MathStatementsSymbol.KIND);

        EMADLCocos.checkAll(componentInstanceSymbol);

        if (architecture.isPresent()){
            String dPath = getDataPath(taggingResolver, EMAComponentSymbol, componentInstanceSymbol);
            architecture.get().setDataPath(dPath);
            architecture.get().setComponentName(EMAComponentSymbol.getFullName());
            generateCNN(fileContents, taggingResolver, componentInstanceSymbol, architecture.get());
        }
        else if (mathStatements.isPresent()){
            generateMathComponent(fileContents, taggingResolver, componentInstanceSymbol, mathStatements.get());
        }
        else {
            generateSubComponents(fileContents, allInstances, taggingResolver, componentInstanceSymbol, symtab);
        }
    }

    private void fixArmadilloImports(List<FileContent> fileContents){
        for (FileContent fileContent : fileContents){
            fileContent.setFileContent(fileContent.getFileContent()
                    .replaceFirst("#include \"armadillo.h\"",
                            "#include \"armadillo\""));
        }
    }

    public void generateCNN(List<FileContent> fileContents, TaggingResolver taggingResolver, EMAComponentInstanceSymbol instance, ArchitectureSymbol architecture){
        Map<String,String> contentMap = cnnArchGenerator.generateStrings(architecture);
        String fullName = instance.getFullName().replaceAll("\\.", "_");

        //get the components execute method
        String executeKey = "execute_" + fullName;
        String executeMethod = contentMap.get(executeKey);
        if (executeMethod == null){
            throw new IllegalStateException("execute method of " + fullName + " not found");
        }
        contentMap.remove(executeKey);

        String component = emamGen.generateString(taggingResolver, instance, (MathStatementsSymbol) null);
        FileContent componentFileContent = new FileContent(
                transformComponent(component, "CNNPredictor_" + fullName, executeMethod),
                instance);

        for (String fileName : contentMap.keySet()){
            fileContents.add(new FileContent(contentMap.get(fileName), fileName));
        }
        fileContents.add(componentFileContent);
        fileContents.add(new FileContent(readResource("CNNTranslator.h", Charsets.UTF_8), "CNNTranslator.h"));
    }

    protected String transformComponent(String component, String predictorClassName, String executeMethod){
        String networkVariableName = "_cnn_";

        //insert includes
        component = component.replaceFirst("using namespace",
                "#include \"" + predictorClassName + ".h" + "\"\n" +
                        "#include \"CNNTranslator.h\"\n" +
                        "using namespace");

        //insert network attribute
        component = component.replaceFirst("public:",
                "public:\n" + predictorClassName + " " + networkVariableName + ";");

        //insert execute method
        component = component.replaceFirst("void execute\\(\\)\\s\\{\\s\\}",
                "void execute(){\n" + executeMethod + "\n}");
        return component;
    }

    public void generateMathComponent(List<FileContent> fileContents, TaggingResolver taggingResolver, EMAComponentInstanceSymbol EMAComponentSymbol, MathStatementsSymbol mathStatementsSymbol){
        fileContents.add(new FileContent(
                emamGen.generateString(taggingResolver, EMAComponentSymbol, mathStatementsSymbol),
                EMAComponentSymbol));
    }

    public void generateSubComponents(List<FileContent> fileContents, Set<EMAComponentInstanceSymbol> allInstances, TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol, Scope symtab){
        fileContents.add(new FileContent(emamGen.generateString(taggingResolver, componentInstanceSymbol, (MathStatementsSymbol) null), componentInstanceSymbol));
        String lastNameWithoutArrayPart = "";
        for (EMAComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {
            int arrayBracketIndex = instanceSymbol.getName().indexOf("[");
            boolean generateComponentInstance = true;
            if (arrayBracketIndex != -1) {
                generateComponentInstance = !instanceSymbol.getName().substring(0, arrayBracketIndex).equals(lastNameWithoutArrayPart);
                lastNameWithoutArrayPart = instanceSymbol.getName().substring(0, arrayBracketIndex);
                Log.info(lastNameWithoutArrayPart, "Without:");
                Log.info(generateComponentInstance + "", "Bool:");
            }
            if (generateComponentInstance) {
                generateComponent(fileContents, allInstances, taggingResolver, instanceSymbol, symtab);
            }
        }
    }

    private String getConfigFilename(String mainComponentName, String componentFullName, String componentName) {
        String trainConfigFilename;
        String mainComponentConfigFilename = mainComponentName.replaceAll("\\.", "/");
        String componentConfigFilename = componentFullName.replaceAll("\\.", "/");
        String instanceConfigFilename = componentFullName.replaceAll("\\.", "/") + "_"  + componentName;
        if (Files.exists(Paths.get( getModelsPath() + instanceConfigFilename + ".cnnt"))) {
            trainConfigFilename = instanceConfigFilename;
        }
        else if (Files.exists(Paths.get( getModelsPath() + componentConfigFilename + ".cnnt"))){
            trainConfigFilename = componentConfigFilename;
        }
        else if (Files.exists(Paths.get( getModelsPath() + mainComponentConfigFilename + ".cnnt"))){
            trainConfigFilename = mainComponentConfigFilename;
        }
        else{
            Log.error("Missing configuration file. " +
                    "Could not find a file with any of the following names (only one needed): '"
                    + getModelsPath() + instanceConfigFilename + ".cnnt', '"
                    + getModelsPath() + componentConfigFilename + ".cnnt', '"
                    + getModelsPath() + mainComponentConfigFilename + ".cnnt'." +
                    " These files denote respectively the configuration for the single instance, the component or the whole system.");
            return null;
        }
        return trainConfigFilename;
    }

    public List<FileContent> generateCNNTrainer(Set<EMAComponentInstanceSymbol> allInstances, String mainComponentName) {
        List<FileContent> fileContents = new ArrayList<>();
        for (EMAComponentInstanceSymbol componentInstance : allInstances) {
            EMAComponentSymbol component = componentInstance.getComponentType().getReferencedSymbol();
            Optional<ArchitectureSymbol> architecture = component.getSpannedScope().resolve("", ArchitectureSymbol.KIND);

            if (architecture.isPresent()) {
                String trainConfigFilename = getConfigFilename(mainComponentName, component.getFullName(), component.getName());

                //should be removed when CNNTrain supports packages
                List<String> names = Splitter.on("/").splitToList(trainConfigFilename);
                trainConfigFilename = names.get(names.size()-1);
                Path modelPath = Paths.get(getModelsPath() + Joiner.on("/").join(names.subList(0,names.size()-1)));
                ConfigurationSymbol configuration = cnnTrainGenerator.getConfigurationSymbol(modelPath, trainConfigFilename);
                cnnTrainGenerator.setInstanceName(componentInstance.getFullName().replaceAll("\\.", "_"));
                Map<String, String> fileContentMap =  cnnTrainGenerator.generateStrings(configuration);
                for (String fileName : fileContentMap.keySet()){
                    fileContents.add(new FileContent(fileContentMap.get(fileName), fileName));
                }
            }
        }
        return fileContents;
    }

    public String readResource(final String fileName, Charset charset) {
        try {
            return Resources.toString(Resources.getResource(fileName), charset);

        } catch (IllegalArgumentException e) {
            System.err.println("Resource file " + fileName + " not found");
            System.exit(1);
            return null;
        } catch (IOException e) {
            System.err.println("IO Error occurred");
            System.exit(1);
            return null;
        }
    }
}
