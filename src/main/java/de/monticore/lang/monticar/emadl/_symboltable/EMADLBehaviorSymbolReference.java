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

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.references.CommonSymbolReference;
import de.monticore.symboltable.references.SymbolReference;

import java.util.Optional;

public class EMADLBehaviorSymbolReference extends EMADLBehaviorSymbol implements SymbolReference<EMADLBehaviorSymbol> {

    protected final SymbolReference<EMADLBehaviorSymbol> reference;

    public EMADLBehaviorSymbolReference(final String name, final Scope enclosingScopeOfReference) {
        super(name);
        reference = new CommonSymbolReference<>(name, EMADLBehaviorSymbol.KIND, enclosingScopeOfReference);
    }

    //SymbolReference methods
    @Override
    public EMADLBehaviorSymbol getReferencedSymbol() {
        return reference.getReferencedSymbol();
    }

    @Override
    public boolean existsReferencedSymbol() {
        return reference.existsReferencedSymbol();
    }

    @Override
    public boolean isReferencedSymbolLoaded() {
        return reference.isReferencedSymbolLoaded();
    }


    //CommonSymbol methods
    @Override
    public String getName() {
        return getReferencedSymbol().getName();
    }

    @Override
    public String getFullName() {
        return getReferencedSymbol().getFullName();
    }

    @Override
    public AccessModifier getAccessModifier() {
        return getReferencedSymbol().getAccessModifier();
    }

    @Override
    public void setAccessModifier(AccessModifier accessModifier) {
        getReferencedSymbol().setAccessModifier(accessModifier);
    }

    @Override
    public void setAstNode(ASTNode node) {
        getReferencedSymbol().setAstNode(node);
    }

    @Override
    public Optional<ASTNode> getAstNode() {
        return getReferencedSymbol().getAstNode();
    }



    //EMADLBehaviorSymbol methods
    @Override
    public ArchitectureConstructorSymbol getArchitectureConstructor() {
        return getReferencedSymbol().getArchitectureConstructor();
    }

    @Override
    public void setArchitectureConstructor(ArchitectureConstructorSymbol architectureConstructor) {
        getReferencedSymbol().setArchitectureConstructor(architectureConstructor);
    }

    @Override
    public ConfigConstructorSymbol getConfigConstructor() {
        return getReferencedSymbol().getConfigConstructor();
    }

    @Override
    public void setConfigConstructor(ConfigConstructorSymbol configConstructor) {
        getReferencedSymbol().setConfigConstructor(configConstructor);
    }

    @Override
    public ArchitectureSymbol resolveArchitecture() {
        return getReferencedSymbol().resolveArchitecture();
    }

    @Override
    public ConfigurationSymbol resolveConfiguration() {
        return getReferencedSymbol().resolveConfiguration();
    }
}
