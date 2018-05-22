package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.middleware.helpers.NameHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

//TODO: make GeneratorCpp implement GeneratorImpl directly!
public class CPPGenImpl implements GeneratorImpl {
    private String generationTargetPath;
    private GeneratorCPP generatorCPP;

    public CPPGenImpl(){
        generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
    }

    public void setGeneratorCPP(GeneratorCPP generatorCPP){
        this.generatorCPP = generatorCPP;
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> files = new ArrayList<>();

        generatorCPP.setGenerationTargetPath(generationTargetPath);
        files.add(generatorCPP.generateFile(generateCMake(componentInstanceSymbol)));
        files.addAll(generatorCPP.generateFiles(componentInstanceSymbol, taggingResolver));

        fixKnownErrors(files);

        return files;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }

    private FileContent generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        FileContent cmake = new FileContent();
        cmake.setFileName("CMakeLists.txt");
        String name = NameHelper.getNameTargetLanguage(componentInstanceSymbol.getFullName());
        cmake.setFileContent(TemplateHelper.getCmakeCppTemplate().replace("${compName}", name));
        return cmake;
    }

    private void fixKnownErrors(List<File> files) throws IOException {
        for (File f : files) {
            Path path = Paths.get(f.getAbsolutePath());
            Charset charset = StandardCharsets.UTF_8;
            String content = new String(Files.readAllBytes(path), charset);

            content = content
                    //Matrix access problems
                    .replace("int curIndex = indexLookup(i);","int curIndex = indexLookup(i-1);")
                    .replace("-1-1", "-1") //not present in emam2cpp master
                    .replace("] ;","];")
            ;

            Files.write(path, content.getBytes(charset));
        }
    }
}
