/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Paths;

public class AbstractSymtabTest {
    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = getModelingLanguageFamily();
        final ModelPath mp = getModelPath(modelPath);

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);

//        LogConfig.init();
        return scope;
    }

    protected static ModelingLanguageFamily getModelingLanguageFamily() {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
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
