package de.monticore.lang.monticar.generator.roscpp;

import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//TODO: better exceptions
public class YamlHelper {

    public static List<File> generateFromFile(String configFilePath, TaggingResolver symtab, GeneratorRosCpp generatorRosCpp) throws IOException {
        TagReader<RosTag> reader = new TagReader<>();
        //TODO: fails silently, rewrite?
        List<RosTag> rosTags = reader.readYAML(configFilePath);
        List<File> result = new ArrayList<>();

        for (RosTag rosTag : rosTags) {
            ResolvedRosTag resolvedRosTag = ResolveHelper.resolveRosTag(rosTag, symtab);
            DataHelper.setCurrentResolvedRosTag(resolvedRosTag);
            result.addAll(generatorRosCpp.generateFiles(resolvedRosTag.getComponent(), symtab));
        }

        return result;
    }
}
