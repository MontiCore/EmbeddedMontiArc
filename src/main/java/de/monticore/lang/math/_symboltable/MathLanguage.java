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
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Collections;

public class MathLanguage extends MathLanguageTOP {

  public MathLanguage() {
    super("MathLanguage", "m");
  }

  @Override
  protected MathModelLoader provideModelLoader() {
    return new MathModelLoader(this);
  }

  public MathSymbolTableCreator getSymbolTableCreator() {
    ResolvingConfiguration resolvingConfiguration = new ResolvingConfiguration();
    resolvingConfiguration.addDefaultFilters(this.getResolvingFilters());
    ArtifactScope scope = new ArtifactScope("unnamed", Collections.EMPTY_LIST);
    return this.getSymbolTableCreator(resolvingConfiguration, scope).get();
  }

}
