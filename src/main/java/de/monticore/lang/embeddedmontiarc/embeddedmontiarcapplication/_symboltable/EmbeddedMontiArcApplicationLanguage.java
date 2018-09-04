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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.EmbeddingModelingLanguage;
import de.monticore.ModelingLanguage;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.application.application._symboltable.ApplicationLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._parser.EmbeddedMontiArcApplicationParser;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class EmbeddedMontiArcApplicationLanguage extends EmbeddingModelingLanguage{
    public static final String FILE_ENDING = "emaapl";
    public static final ModelingLanguage HOST_LANGUAGE =
            new EmbeddedMontiArcLanguage();
    public static final ModelingLanguage EMBEDDED_LANGUAGE =
            new ApplicationLanguage();


    public EmbeddedMontiArcApplicationLanguage() {
        super("Embedded MontiArc Math Language", FILE_ENDING,
                HOST_LANGUAGE, EMBEDDED_LANGUAGE);
    }

    @Override
    public Collection<ResolvingFilter<? extends Symbol>> getResolvingFilters() {
        List<ResolvingFilter<? extends Symbol>> ret =
                new ArrayList<>(super.getResolvingFilters());
        //ret.add(new ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter());
        //ret.add(new PortArraySymbol2MathVariableDeclarationSymbolTypeFilter());
        // ret.add(new PortSymbol2MathVariableDeclarationTypeFilter());
        return ret;
    }

    @Override
    protected EmbeddedMontiArcApplicationModelLoader provideModelLoader() {
        return new EmbeddedMontiArcApplicationModelLoader(this);
    }

    @Override
    public MCConcreteParser getParser() {
        return new EmbeddedMontiArcApplicationParser();
    }

    @Override
    public Optional<EmbeddedMontiArcApplicationSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        return Optional.of(new EmbeddedMontiArcApplicationSymbolTableCreator(
                resolvingConfiguration, enclosingScope));
    }

}
