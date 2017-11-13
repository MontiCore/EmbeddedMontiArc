package de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema;

import de.monticore.lang.montiarc.montiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.montiarc.tagging._ast.ASTNameScope;
import de.monticore.lang.montiarc.tagging._ast.ASTScope;
import de.monticore.lang.montiarc.tagging._ast.ASTTag;
import de.monticore.lang.montiarc.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.montiarc.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.montiarc.tagging.helper.NumericLiteral;
import de.monticore.lang.montiarc.tagvalue._ast.ASTNumericTagValue;
import de.monticore.lang.montiarc.tagvalue._parser.TagValueParser;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.monticore.types.types._ast.ASTComplexArrayType;
import de.monticore.types.types._ast.ASTPrimitiveArrayType;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Created by ernst on 15.07.2016.
 */
public class TagBreakpointsSymbolCreator implements TagSymbolCreator {

    public static Scope getGlobalScope(final Scope scope) {
        Scope s = scope;
        while (s.getEnclosingScope().isPresent()) {
            s = s.getEnclosingScope().get();
        }
        return s;
    }

    public void create(ASTTaggingUnit unit, Scope gs) {
        if (unit.getQualifiedNames().stream()
                .map(q -> q.toString())
                .filter(n -> n.endsWith("TagBreakpointsTagSchema"))
                .count() == 0) {
            return; // the tagging model is not conform to the TagExecutionOrderTagSchema tagging schema
        }
        final String packageName = Joiners.DOT.join(unit.getPackage());
        final String rootCmp = // if-else does not work b/cpp of final (required by streams)
                (unit.getTagBody().getTargetModel().isPresent()) ?
                        Joiners.DOT.join(packageName,
                                unit.getTagBody().getTargetModel().get()
                                        .getQualifiedNameString()) :
                        packageName;

        handleTagElements(unit, gs, rootCmp);
    }

    protected void handleTagElements(ASTTaggingUnit unit, Scope gs, String rootCmp) {
        for (ASTTag element : unit.getTagBody().getTags()) {
            element.getTagElements().stream()
                    .filter(t -> t.getName().equals("TagBreakpoints"))
                    .filter(t -> t.getTagValue().isPresent())
                    .map(t -> checkContent(t.getTagValue().get()))
                    .filter(v -> v != null)
                    .forEachOrdered(v ->
                            element.getScopes().stream()
                                    .filter(this::checkScope)
                                    .map(s -> (ASTNameScope) s)
                                    .map(s -> getGlobalScope(gs).<Symbol>resolveDownMany(
                                            Joiners.DOT.join(rootCmp, s.getQualifiedName().toString()),
                                            SymbolKind.KIND))
                                    .filter(s -> !s.isEmpty())
                                    .map(this::checkKind)
                                    .filter(s -> s != null)
                                    .forEachOrdered(s -> s.addTag(new TagBreakpointsSymbol(v.toArray(new Double[v.size()])))));
        }
    }

    protected Collection<Double> checkContent(String s) {
        TagValueParser parser = new TagValueParser();
        Collection<Optional<ASTNumericTagValue>> ast = new ArrayList<>();
        try {
            boolean enableFailQuick = Log.isFailQuickEnabled();
            Log.enableFailQuick(false);
            long errorCount = Log.getErrorCount();
            String[] parts = s.split(",");
            for (String i : parts)
                ast.add(parser.parseString_NumericTagValue(i));
            Log.enableFailQuick(enableFailQuick);
            if (Log.getErrorCount() > errorCount) {
                throw new Exception("Error occured during parsing.");
            }
        } catch (Exception e) {
            Log.error(String.format("0xT0010 Could not parse %s with TagValueParser#parseString_NumericTagValue.",
                    s), e);
            return null;
        }
        if (ast.isEmpty()) {
            return null;
        }
        return ast.stream().map(n -> (NumericLiteral.getValue(n.get().getNumericLiteral())).doubleValue())
                .collect(Collectors.toList());
    }

    protected ExpandedComponentInstanceSymbol checkKind(Collection<Symbol> symbols) {
        ExpandedComponentInstanceSymbol ret = null;
        for (Symbol symbol : symbols) {
            if (symbol.getKind().isSame(ExpandedComponentInstanceSymbol.KIND)) {
                if (ret != null) {
                    Log.error(String.format("0xT0011 Found more than one symbol: '%s' and '%s'",
                            ret, symbol));
                    return null;
                }
                ret = (ExpandedComponentInstanceSymbol) symbol;
            }
        }
        if (ret == null) {
            Log.error(String.format("0xT0012 Invalid symbol kinds: %s. tagTypeName expects as symbol kind 'ExpandedComponentInstanceSymbol.KIND'.",
                    symbols.stream().map(s -> "'" + s.getKind().toString() + "'").collect(Collectors.joining(", "))));
            return null;
        }
        return ret;
    }

    protected boolean checkScope(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            return true;
        }
        Log.error(String.format("0xT0013 Invalid scope kind: '%s'. PowerConsumption expects as scope kind 'NameScope'.",
                scope.getScopeKind()), scope.get_SourcePositionStart());
        return false;
    }

}
