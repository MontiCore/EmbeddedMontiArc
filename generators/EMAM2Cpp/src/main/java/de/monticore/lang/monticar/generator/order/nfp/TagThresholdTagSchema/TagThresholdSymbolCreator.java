/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
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
public class TagThresholdSymbolCreator implements TagSymbolCreator {

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
                .filter(n -> n.endsWith("TagThresholdTagSchema"))
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
                    .filter(t -> t.getName().equals("TagThreshold"))
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
                                    .forEachOrdered(s -> gs.addTag(s, new TagThresholdSymbol(v.doubleValue()))));
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
            Log.error(String.format("0xT0030 Could not parse %s with TagValueParser#parse_StringNumericTagValue.",
                    s), e);
            return null;
        }
        if (!ast.isPresent()) {
            return null;
        }
        return NumericLiteral.getValue(ast.get().getNumericLiteral());
    }

    protected EMAComponentInstanceSymbol checkKind(Collection<Symbol> symbols) {
        EMAComponentInstanceSymbol ret = null;
        for (Symbol symbol : symbols) {
            if (symbol.getKind().isSame(EMAComponentInstanceSymbol.KIND)) {
                if (ret != null) {
                    Log.error(String.format("0xT0031 Found more than one symbol: '%s' and '%s'",
                            ret, symbol));
                    return null;
                }
                ret = (EMAComponentInstanceSymbol) symbol;
            }
        }
        if (ret == null) {
            Log.error(String.format("0xT0032 Invalid symbol kinds: %s. tagTypeName expects as symbol kind 'ExpandedComponentInstanceSymbol.KIND'.",
                    symbols.stream().map(s -> "'" + s.getKind().toString() + "'").collect(Collectors.joining(", "))));
            return null;
        }
        return ret;
    }

    protected boolean checkScope(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            return true;
        }
        Log.error(String.format("0xT0033 Invalid scope kind: '%s'. PowerConsumption expects as scope kind 'NameScope'.",
                scope.getScopeKind()), scope.get_SourcePositionStart());
        return false;
    }

    public void create(ASTTaggingUnit astTaggingUnit, Scope scope) {
        //TODO implement me, required by newer version
    }
}
