/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Paths;

public final class Utils {
    private Utils() {
        // utility class -- do not instantiate
    }

    public static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        ModelPath mp = new ModelPath();
        if (modelPath != null && modelPath.length > 0) {
            for (String m : modelPath) {
                mp.addEntry(Paths.get(m));
            }
        }
        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }
}
