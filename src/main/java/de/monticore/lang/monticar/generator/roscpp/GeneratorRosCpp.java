package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.FormatHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.PrinterHelper;
import de.monticore.lang.monticar.generator.roscpp.util.AdapterBluePrint;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
        this.generationTargetPath = generationTargetPath.endsWith("/") ? generationTargetPath : generationTargetPath + "/";
    }

    public List<File> generateFiles(EMAComponentInstanceSymbol component, TaggingResolver symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(component);

        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {
            files.add(PrinterHelper.generateFile(fileContent, getGenerationTargetPath()));
        }

        return files;
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
        FileContent adapter = new FileContent();

        LanguageUnitRosCppAdapter languageUnitRosCppAdapter = new LanguageUnitRosCppAdapter(this.isRos2Mode());
        Optional<AdapterBluePrint> bluePrintOpt = languageUnitRosCppAdapter.generateBluePrint(component);

        if (bluePrintOpt.isPresent()) {
            bluePrintOpt.get().getAdapterFileContent(component, adapter);
            if (generateCMake) {
                res.addAll(LanguageUnitRosCMake.generateFiles(component, languageUnitRosCppAdapter.getRosInterfaces(), isRos2Mode()));
            }
            res.add(adapter);
        }
        return res;
    }

}
