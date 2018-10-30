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
package de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.tagging._ast.ASTNameScope;
import de.monticore.lang.tagging._ast.ASTScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.tagging.helper.NumericLiteral;
import de.monticore.lang.tagvalue._ast.ASTNumericTagValue;
import de.monticore.lang.tagvalue._parser.TagValueParser;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Created by ernst on 15.07.2016.
 */
public class TagInitSymbolCreator implements TagSymbolCreator {

    public static Scope getGlobalScope(final Scope scope) {
        Scope s = scope;
        while (s.getEnclosingScope().isPresent()) {
            s = s.getEnclosingScope().get();
        }
        return s;
    }

    public void create(ASTTaggingUnit unit, TaggingResolver gs) {
        if (unit.getQualifiedNameList().stream()
                .map(q -> q.toString())
                .filter(n -> n.endsWith("TagInitTagSchema"))
                .count() == 0) {
            return; // the tagging model is not conform to the TagExecutionOrderTagSchema tagging schema
        }
        final String packageName = Joiners.DOT.join(unit.getPackageList());
        final String rootCmp = // if-else does not work b/cpp of final (required by streams)
                (unit.getTagBody().getTargetModelOpt().isPresent()) ?
                        Joiners.DOT.join(packageName,
                                unit.getTagBody().getTargetModelOpt().get()
                                        .getQualifiedNameString()) :
                        packageName;

        for (ASTTag element : unit.getTagBody().getTagList()) {
            element.getTagElementList().stream()
                    .filter(t -> t.getName().equals("TagInit"))
                    .filter(t -> t.getTagValueOpt().isPresent())
                    .map(t -> checkContent(t.getTagValueOpt().get()))
                    .filter(v -> v != null)
                    .forEachOrdered(v ->
                            element.getScopeList().stream()
                                    .filter(this::checkScope)
                                    .map(s -> (ASTNameScope) s)
                                    .map(s -> getGlobalScope(gs).<Symbol>resolveDownMany(
                                            Joiners.DOT.join(rootCmp, s.getQualifiedName().toString()),
                                            SymbolKind.KIND))
                                    .filter(s -> !s.isEmpty())
                                    .map(this::checkKind)
                                    .filter(s -> s != null)
                                    .forEachOrdered(s -> gs.addTag(s, new TagInitSymbol(v.doubleValue()))));
        }
    }

    protected Number checkContent(String s) {
        TagValueParser parser = new TagValueParser();
        Optional<ASTNumericTagValue> ast;
        try {
            boolean enableFailQuick = Log.isFailQuickEnabled();
            Log.enableFailQuick(false);
            long errorCount = Log.getErrorCount();

            ast = parser.parse_StringNumericTagValue(s);

            Log.enableFailQuick(enableFailQuick);
            if (Log.getErrorCount() > errorCount) {
                throw new Exception("Error occured during parsing.");
            }
        } catch (Exception e) {
            Log.error(String.format("0xT0018 Could not parse %s with TagValueParser#parse_StringNumericTagValue.",
                    s), e);
            return null;
        }
        if (!ast.isPresent()) {
            return null;
        }
        return NumericLiteral.getValue(ast.get().getNumericLiteral());
    }

    protected ExpandedComponentInstanceSymbol checkKind(Collection<Symbol> symbols) {
        ExpandedComponentInstanceSymbol ret = null;
        for (Symbol symbol : symbols) {
            if (symbol.getKind().isSame(ExpandedComponentInstanceSymbol.KIND)) {
                if (ret != null) {
                    Log.error(String.format("0xT0019 Found more than one symbol: '%s' and '%s'",
                            ret, symbol));
                    return null;
                }
                ret = (ExpandedComponentInstanceSymbol) symbol;
            }
        }
        if (ret == null) {
            Log.error(String.format("0xT0020 Invalid symbol kinds: %s. tagTypeName expects as symbol kind 'ExpandedComponentInstanceSymbol.KIND'.",
                    symbols.stream().map(s -> "'" + s.getKind().toString() + "'").collect(Collectors.joining(", "))));
            return null;
        }
        return ret;
    }

    protected boolean checkScope(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            return true;
        }
        Log.error(String.format("0xT0021 Invalid scope kind: '%s'. PowerConsumption expects as scope kind 'NameScope'.",
                scope.getScopeKind()), scope.get_SourcePositionStart());
        return false;
    }


    public void create(ASTTaggingUnit astTaggingUnit, Scope scope) {
        //TODO implement me, required by newer version
    }
}
