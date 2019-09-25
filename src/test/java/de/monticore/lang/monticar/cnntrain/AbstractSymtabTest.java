/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Paths;

public class AbstractSymtabTest {
    protected static Scope createSymTab(String... modelPath) {
        Log.enableFailQuick(false);
        ModelingLanguageFamily fam = new ModelingLanguageFamily();

        fam.addModelingLanguage(new CNNTrainLanguage());

        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        GlobalScope scope = new GlobalScope(mp, fam);

        return scope;
    }
}
