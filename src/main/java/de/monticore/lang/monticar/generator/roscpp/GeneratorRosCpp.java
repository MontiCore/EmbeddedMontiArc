package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.helper.FormatHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.PrinterHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class GeneratorRosCpp {

    private String generationTargetPath;
    private boolean generateCMake = false;

    public void setGenerateCMake(boolean generateCMake) {
        this.generateCMake = generateCMake;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public List<File> generateFiles(ExpandedComponentInstanceSymbol component, TaggingResolver symtab) throws IOException {
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

    public List<FileContent> generateStrings(ExpandedComponentInstanceSymbol component) {
        List<FileContent> fileContents = new ArrayList<>();
        fileContents.addAll(generateRosAdapter(component));
        fileContents.forEach(fc -> fc.setFileContent(FormatHelper.fixIndentation(fc.getFileContent())));
        return fileContents;
    }

    private List<FileContent> generateRosAdapter(ExpandedComponentInstanceSymbol component) {
        List<FileContent> res = new ArrayList<>();

        FileContent apdapter = new FileContent();

        LanguageUnitRosCppAdapter languageUnitRosCppAdapter = new LanguageUnitRosCppAdapter();
        languageUnitRosCppAdapter.generateBluePrints(component);
        //TODO: unsave, does not work with multiple
        BluePrintCPP currentBluePrint = languageUnitRosCppAdapter.getBluePrints().get(0);

        String classname = currentBluePrint.getName();
        apdapter.setFileName(classname + ".h");
        apdapter.setFileContent(PrinterHelper.printClass(currentBluePrint, ": public IAdapter"));

        if (generateCMake) {
            LanguageUnitRosCMake languageUnitRosCMake = new LanguageUnitRosCMake();
            FileContent cmake = languageUnitRosCMake.generate(component, languageUnitRosCppAdapter.getAdditionalPackages());
            res.add(cmake);
        }

        res.add(apdapter);
        return res;
    }

}
