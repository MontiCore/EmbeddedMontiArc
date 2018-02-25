package de.monticore.lang.monticar.generator.middleware.impls;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.rosmsg.GeneratorRosMsg;
import de.monticore.lang.monticar.generator.rosmsg.RosMsg;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//TODO: make GeneratorRosMsg implement GeneratorImpl
public class RosMsgImpl implements GeneratorImpl {
    private GeneratorRosMsg generatorRosMsg;
    private List<MCTypeReference<? extends MCTypeSymbol>> rosTypesToGenerate = new ArrayList<>();
    private String packageName;

    public RosMsgImpl(String packageName) {
        this.packageName = packageName;
        generatorRosMsg = new GeneratorRosMsg();
    }

    @Override
    public List<File> generate(ExpandedComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver) throws IOException {
        List<File> files = new ArrayList<>();
        for (MCTypeReference<? extends MCTypeSymbol> type : rosTypesToGenerate) {
            files.addAll(generatorRosMsg.generate(type));
        }
        return files;
    }

    @Override
    public void setGenerationTargetPath(String path) {
        generatorRosMsg.setTarget(path, packageName);
    }

    public void addRosTypeToGenerate(MCTypeReference<? extends MCTypeSymbol> typeReference) {
        rosTypesToGenerate.add(typeReference);
    }

    public RosMsg getRosType(MCTypeReference<? extends MCTypeSymbol> typeReference) {
        return generatorRosMsg.getRosType(typeReference);
    }
}
