package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.middleware.helpers.FileHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.NameHelper;
import de.monticore.lang.monticar.generator.middleware.helpers.TemplateHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DummyMiddlewareGenImpl implements GeneratorImpl {

    private String generationTargetPath;

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {

        List<File> res = new ArrayList<>();
        res.add(FileHelper.generateFile(generationTargetPath, generateCMake(componentInstanceSymbol)));
        res.add(FileHelper.generateFile(generationTargetPath, generateAdapter(componentInstanceSymbol)));
        return res;
    }


    private FileContent generateAdapter(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        String name = NameHelper.getNameTargetLanguage(componentInstanceSymbol.getFullName());
        String content = TemplateHelper.getDummyAdapterTemplate()
                .replace("${compName}", name);

        FileContent res = new FileContent();
        res.setFileName("DummyAdapter_" + name + ".h");
        res.setFileContent(content);
        return res;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        this.generationTargetPath = path;
    }

    @Override
    public boolean willAccept(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        return componentInstanceSymbol.getPortsList().stream()
                .map(PortSymbol::getMiddlewareSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .filter(mws -> mws.isKindOf(DummyMiddlewareSymbol.KIND))
                .count() > 0;
    }

    private FileContent generateCMake(ExpandedComponentInstanceSymbol componentInstanceSymbol) {
        FileContent res = new FileContent();
        String name = NameHelper.getNameTargetLanguage(componentInstanceSymbol.getFullName());
        String content = TemplateHelper.getDummyCmakeTemplate()
                .replace("${compName}", name);

        res.setFileName("CMakeLists.txt");
        res.setFileContent(content);
        return res;
    }
}
