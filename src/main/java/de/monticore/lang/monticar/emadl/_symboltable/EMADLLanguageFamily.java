/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
