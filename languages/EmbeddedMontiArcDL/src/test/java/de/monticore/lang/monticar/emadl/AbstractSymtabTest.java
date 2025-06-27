/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Paths;

public class AbstractSymtabTest {
    protected static Scope createSymTab(String... modelPath) {
        ConstantPortHelper.resetLastID();
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        EMADLLanguage montiArcLanguage = new EMADLLanguage();

        fam.addModelingLanguage(montiArcLanguage);
        fam.addModelingLanguage(new StreamUnitsLanguage());
        fam.addModelingLanguage(new StructLanguage());
        fam.addModelingLanguage(new EnumLangLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        // LogConfig.init(); //TODO comment OUT for debug output

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }
}
