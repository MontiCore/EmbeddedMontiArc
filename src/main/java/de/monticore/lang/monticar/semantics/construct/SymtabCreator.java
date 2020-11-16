/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;

import java.nio.file.Paths;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.LinkedList;

public class SymtabCreator {

    public static TaggingResolver createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = getStandardModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());

        return create(fam, modelPath);
    }

    protected static TaggingResolver create(ModelingLanguageFamily fam, String... modelPath) {
        final ModelPath mp = getModelPath(modelPath);

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);

        LogConfig.init();
        return new TaggingResolver(scope, Arrays.asList(modelPath.clone()));
    }

    protected static ModelingLanguageFamily getStandardModelingLanguageFamily() {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EventLanguage());
        return fam;
    }

    protected static ModelPath getModelPath(String... modelPath) {
        ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        return mp;
    }
}
