/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTEMVCompilationUnit;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiViewModelLoaderTOP;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Strings.isNullOrEmpty;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 * @author Michael von Wenckstern
 */
public class EmbeddedMontiViewModelLoader
    extends EmbeddedMontiViewModelLoaderTOP {

  public EmbeddedMontiViewModelLoader(EmbeddedMontiViewLanguage language) {
    super(language);
  }

  /**
   * this method should be implemented into the ModelPath class
   */
  private static Collection<Path> getEntriesFromModelPath(ModelPath modelPath) {
    String s = modelPath.toString().replace("[", "").replace("]", "").replace(" ", "");
    String ss[] = s.split(",");
    return Arrays.stream(ss).map(str -> Paths.get(URI.create(str))).collect(Collectors.toSet());
  }

  @Override
  public Collection<ASTEMVCompilationUnit> loadModelsIntoScope(final String qualifiedModelName, final ModelPath modelPath, final MutableScope enclosingScope, final ResolvingConfiguration ResolvingConfiguration) {

    final Collection<ASTEMVCompilationUnit> asts = loadModels(qualifiedModelName, modelPath);

    for (ASTEMVCompilationUnit ast : asts) {
      createSymbolTableFromAST(ast, qualifiedModelName, enclosingScope, ResolvingConfiguration);
    }

    return asts;
  }
}
