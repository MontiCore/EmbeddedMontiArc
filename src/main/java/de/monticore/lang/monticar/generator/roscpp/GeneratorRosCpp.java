package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.monticore.lang.monticar.generator.roscpp.util.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.helper.FormatHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.NameHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.PrinterHelper;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

public class GeneratorRosCpp {

    private String generationTargetPath;
    private boolean generateCMake = false;
    private boolean ros2Mode = false;

    public boolean isRos2Mode() {
        return ros2Mode;
    }

    public void setRos2Mode(boolean ros2Mode) {
        this.ros2Mode = ros2Mode;
    }

    public void setGenerateCMake(boolean generateCMake) {
        this.generateCMake = generateCMake;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public List<File> generateFiles(EMAComponentInstanceSymbol component, TaggingResolver symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(component);

        if (getGenerationTargetPath().charAt(getGenerationTargetPath().length() - 1) != '/') {
            setGenerationTargetPath(getGenerationTargetPath() + "/");
        }
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {

            files.add(generateFile(fileContent));
        }

        return files;
    }

    public File generateFile(FileContent fileContent) throws IOException {
        File f = new File(getGenerationTargetPath() + fileContent.getFileName());
        Log.info(f.getName(), "FileCreation:");
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if (!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(f));
        bufferedWriter.write(fileContent.getFileContent(), 0, fileContent.getFileContent().length());
        bufferedWriter.close();
        return f;
    }

    public List<FileContent> generateStrings(EMAComponentInstanceSymbol component) {
        List<FileContent> fileContents = new ArrayList<>();
        fileContents.addAll(generateRosAdapter(component));
        fileContents.stream()
                .filter(fc -> fc.getFileName().endsWith(".h"))
                .forEach(fc -> fc.setFileContent(FormatHelper.fixIndentation(fc.getFileContent())));
        return fileContents;
    }

    private List<FileContent> generateRosAdapter(EMAComponentInstanceSymbol component) {
        List<FileContent> res = new ArrayList<>();

        FileContent apdapter = new FileContent();

        LanguageUnitRosCppAdapter languageUnitRosCppAdapter = new LanguageUnitRosCppAdapter();
        languageUnitRosCppAdapter.setRos2Mode(this.isRos2Mode());
        Optional<BluePrintCPP> currentBluePrint = languageUnitRosCppAdapter.generateBluePrint(component);

        if(currentBluePrint.isPresent()) {

            String classname = currentBluePrint.get().getName();
            apdapter.setFileName(classname + ".h");
            String nameTargetLanguage = NameHelper.getComponentNameTargetLanguage(component.getFullName());
            apdapter.setFileContent(PrinterHelper.printClass(currentBluePrint.get(), ": public IAdapter_" + nameTargetLanguage));

            GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
            generatorRosMsg.setRos2mode(ros2Mode);
            for (Map.Entry<RosMsg, MCTypeReference<? extends MCTypeSymbol>> entry : languageUnitRosCppAdapter.getUsedRosMsgs().entrySet()) {
                String packageName = Arrays.stream(entry.getKey().getName().split("/")).findFirst().get();
                if (packageName.equals("struct_msgs")) {
                    String ros2extra = isRos2Mode() ? "msg/" : "";
                    generatorRosMsg.setTarget(generationTargetPath + "/" + ros2extra + packageName, packageName);
                    List<FileContent> tmpFileContents = generatorRosMsg.generateStrings(entry.getValue());
                    tmpFileContents.forEach(fc -> fc.setFileName(packageName + "/" + ros2extra + fc.getFileName()));
                    if(ros2Mode){
                        lowercaseMsgFieldNames(tmpFileContents);
                    }
                    res.addAll(tmpFileContents);
                }
            }

            if (generateCMake) {
                LanguageUnitRosCMake languageUnitRosCMake = new LanguageUnitRosCMake();

                List<RosMsg> rosMsgs = new ArrayList<>(languageUnitRosCppAdapter.getUsedRosMsgs().keySet());
                res.addAll(languageUnitRosCMake.generate(component, languageUnitRosCppAdapter.getAdditionalPackages(), rosMsgs, isRos2Mode()));
            }

            res.add(apdapter);
        }
        return res;
    }

    private void lowercaseMsgFieldNames(List<FileContent> tmpFileContents) {
        tmpFileContents.forEach(fc ->{
            String fixedFileContent = Arrays.stream(fc.getFileContent().split("\\r?\\n"))
                    .map(line -> line.substring(0,line.indexOf(" ")) + line.substring(line.indexOf(" ")).toLowerCase())
                    .collect(Collectors.joining("\n"));
            fc.setFileContent(fixedFileContent);
        });
    }

}
