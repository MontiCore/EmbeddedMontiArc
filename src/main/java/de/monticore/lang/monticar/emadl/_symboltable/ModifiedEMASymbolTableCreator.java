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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

public class ModifiedEMASymbolTableCreator extends EmbeddedMontiArcSymbolTableCreator {

    private ModifiedExpandedInstanceSymbolCreator instanceSymbolCreator = new ModifiedExpandedInstanceSymbolCreator();

    public ModifiedEMASymbolTableCreator(ResolvingConfiguration resolverConfig, MutableScope enclosingScope) {
        super(resolverConfig, enclosingScope);
    }

    public ModifiedEMASymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    public ModifiedExpandedInstanceSymbolCreator getInstanceSymbolCreator() {
        return instanceSymbolCreator;
    }

    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        this.removeCurrentScope();
        if (!this.aboartVisitComponent) {
            Log.debug("endVisit of " + node.getComponent().getSymbol().get().getFullName(), "SymbolTableCreator:");
            getInstanceSymbolCreator().createInstances((ComponentSymbol)(Log.errorIfNull(node.getComponent().getSymbol().orElse(null))));
        }
    }

}
