/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;

public class EMADLLanguageFamily extends ModelingLanguageFamily {

    public final static EMADLLanguage EMADL_LANGUAGE = new EMADLLanguage();
    public final static EmbeddedMontiArcLanguage EMA_LANGUAGE = new EmbeddedMontiArcLanguage();
    public final static EMADLLanguageFamily INSTANCE = new EMADLLanguageFamily();

    public EMADLLanguageFamily() {
        addModelingLanguage(EMADL_LANGUAGE);
        //addModelingLanguage(EMA_LANGUAGE);
        addModelingLanguage(EMADLLanguage.CNNARCH_LANGUAGE);
        //addModelingLanguage(EMADLLanguage.MATH_LANGUAGE);

        addModelingLanguage(new StreamLanguage());
        addModelingLanguage(new StructLanguage());
    }
}
