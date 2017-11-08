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

import com.google.common.collect.ImmutableSet;
import de.monticore.EmbeddingModelingLanguage;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.montiarc.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.montiarc.tagging._symboltable.TagableModelingLanguage;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.*;

public class EMADLLanguage extends EmbeddingModelingLanguage implements TagableModelingLanguage {

    public static final String FILE_ENDING = "emadl";

    public static final EmbeddedMontiArcLanguage HOST_LANGUAGE = new EmbeddedMontiArcLanguage();
    public static final CNNArchLanguage ARCHITECTURE_LANGUAGE = new CNNArchLanguage();
    public static final CNNTrainLanguage TRAINING_LANGUAGE = new CNNTrainLanguage();


    protected LinkedHashSet<TagSymbolCreator> tagSymbolCreators = new LinkedHashSet<>();

    public EMADLLanguage() {
        super("Embedded MontiArc Deep Learning Language", FILE_ENDING, HOST_LANGUAGE,
                Arrays.asList(ARCHITECTURE_LANGUAGE, TRAINING_LANGUAGE));
    }


    @Override
    public Collection<ResolvingFilter<? extends Symbol>> getResolvingFilters() {
        List<ResolvingFilter<? extends Symbol>> ret =
                new ArrayList<>(super.getResolvingFilters());
        //ret.add(new ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter());
        //ret.add(new PortArraySymbol2MathVariableDeclarationSymbolTypeFilter());
        //ret.add(new PortSymbol2MathVariableDeclarationTypeFilter());
        return ret;
    }


    @Override
    protected EMADLModelLoader provideModelLoader() {
        return new EMADLModelLoader(this);
    }

    @Override
    public MCConcreteParser getParser() {
        return new EMADLParser();
    }

    @Override
    public Optional<EMADLSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        return Optional.of(new EMADLSymbolTableCreator(
                resolvingConfiguration, enclosingScope));
    }

    public void addTagSymbolCreator(TagSymbolCreator tagSymbolCreator) {
        this.tagSymbolCreators.add(tagSymbolCreator);
    }

    public ImmutableSet<TagSymbolCreator> getTagSymbolCreators() {
        return ImmutableSet.copyOf(this.tagSymbolCreators);
    }


}
