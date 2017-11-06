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
package de.monticore.lang.monticar.si._symboltable;

import de.monticore.lang.monticar.resolution._ast.ASTResolution;
import de.monticore.symboltable.CommonSymbol;

/**
 * Created by Sascha on 14.05.2017.
 */
public class ResolutionDeclarationSymbol extends CommonSymbol {
    public static final ResolutionDeclarationSymbolKind KIND = ResolutionDeclarationSymbolKind.INSTANCE;
    protected ASTResolution astResolution;
    protected String nameToResolve;

    public ResolutionDeclarationSymbol(String name, String nameToResolve, ASTResolution astResolution) {
        super(name, KIND);
        this.nameToResolve = nameToResolve;
        this.astResolution = astResolution;
    }

    public ASTResolution getASTResolution() {
        return this.astResolution;
    }

    public String getNameToResolve() {
        return this.nameToResolve;
    }
}
