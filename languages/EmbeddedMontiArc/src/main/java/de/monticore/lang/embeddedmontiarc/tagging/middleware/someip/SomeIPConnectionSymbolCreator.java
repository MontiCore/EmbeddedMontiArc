/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.someip;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.tagging._ast.*;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class SomeIPConnectionSymbolCreator implements TagSymbolCreator {

    /**
     * regular expression pattern:
     * {serviceID = {serviceID}, instanceID = {instanceID}, eventgroupID = {eventgroupID}}
     * to test the pattern just enter:
     * \s*\{\s*serviceid\s*=\s*([0-9]*)\s*,\s*instanceid\s*=\s*([0-9]*)\s*,\s*eventgroupid\s*=\s*([0-9]*)\s*\}\s*
     * at http://www.regexplanet.com/advanced/java/index.html
     */

    public static final Pattern pattern = Pattern.compile("\\s*\\{\\s*serviceID\\s*=\\s*([0-9]*)\\s*,\\s*instanceID\\s*=\\s*([0-9]*)\\s*,\\s*eventgroupID\\s*=\\s*([0-9]*)\\s*\\}\\s*");

    public static Scope getGlobalScope(final Scope scope) {
        Scope s = scope;
        while (s.getEnclosingScope().isPresent()) {
            s = s.getEnclosingScope().get();
        }
        return s;
    }

    public void create(ASTTaggingUnit unit, TaggingResolver tagging) {
        if (unit.getQualifiedNameList().stream()
                .map(q -> q.toString())
                .filter(n -> n.endsWith("SomeIPToEmamTagSchema"))
                .count() == 0) {
            return; // the tagging model is not conform to the traceability tagging schema
        }
        final String packageName = Joiners.DOT.join(unit.getPackageList());
        final String rootCmp = // if-else does not work b/c of final (required by streams)
                (unit.getTagBody().getTargetModelOpt().isPresent()) ?
                        Joiners.DOT.join(packageName, ((ASTNameScope) unit.getTagBody().getTargetModelOpt().get())
                                .getQualifiedNameString()) :
                        packageName;

        for (ASTTag element : unit.getTagBody().getTagList()) {
            List<ASTTagElement> tagElements = element.getTagElementList().stream()
                    .filter(t -> t.getName().equals("SomeIPConnection"))
                    .collect(Collectors.toList());
            // after that point we can throw error messages
            List<Symbol> ports = element.getScopeList().stream()
                    .filter(this::checkScope)
                    .map(s -> (ASTNameScope) s)
                    .map(s -> tagging.resolve(Joiners.DOT.join(rootCmp, // resolve down does not try to reload symbol
                            s.getQualifiedNameString()), EMAPortSymbol.KIND))
                    .filter(Optional::isPresent) // if the symbol is not present, does not mean that the symbol
                    .map(Optional::get)          // is not available at all, maybe it will be loaded later
                    .collect(Collectors.toList());

            List<Symbol> portInstances = element.getScopeList().stream()
                    .filter(this::checkScope)
                    .map(s -> (ASTNameScope) s)
                    .map(s -> tagging.resolve(Joiners.DOT.join(rootCmp, // resolve down does not try to reload symbol
                            s.getQualifiedNameString()), EMAPortInstanceSymbol.KIND))
                    .filter(Optional::isPresent) // if the symbol is not present, does not mean that the symbol
                    .map(Optional::get)          // is not available at all, maybe it will be loaded later
                    .collect(Collectors.toList());

            List<Symbol> taggedSymbols = new ArrayList<>(ports);
            taggedSymbols.addAll(portInstances);

            //Empty tags
            tagElements.stream()
                    .filter(t -> !t.getTagValueOpt().isPresent())
                    .forEachOrdered(tag -> {
                        taggedSymbols.stream()
                                .forEachOrdered(s -> {
                                    SomeIPConnectionSymbol tmpSymbol = new SomeIPConnectionSymbol();
                                    tagging.addTag(s, tmpSymbol);
                                    if (s.isKindOf(EMAPortSymbol.KIND)) {
                                        EMAPortSymbol p = (EMAPortSymbol) s;
                                        p.setMiddlewareSymbol(tmpSymbol);
                                    }else if(s.isKindOf(EMAPortInstanceSymbol.KIND)){
                                        EMAPortInstanceSymbol p = (EMAPortInstanceSymbol) s;
                                        p.setMiddlewareSymbol(tmpSymbol);
                                    }
                                });
                    });

            //Tags with TagValue
            tagElements.stream()
                    .filter(t -> t.getTagValueOpt().isPresent())
                    .map(t -> matchRegexPattern(t.getTagValueOpt().get()))
                    .filter(r -> r != null)
                    .forEachOrdered(m ->
                            taggedSymbols.stream()
                                    .forEachOrdered(s -> {
                                        SomeIPConnectionSymbol tmpSymbol = new SomeIPConnectionSymbol(Integer.parseInt(m.group(1)), Integer.parseInt(m.group(2)), Integer.parseInt(m.group(3)));
                                        tagging.addTag(s, tmpSymbol);
                                        if (s.isKindOf(EMAPortSymbol.KIND)) {
                                            EMAPortSymbol p = (EMAPortSymbol) s;
                                            p.setMiddlewareSymbol(tmpSymbol);
                                        }else if(s.isKindOf(EMAPortInstanceSymbol.KIND)){
                                            EMAPortInstanceSymbol p = (EMAPortInstanceSymbol) s;
                                            p.setMiddlewareSymbol(tmpSymbol);
                                        }
                                    }));
        }
    }

    protected Matcher matchRegexPattern(String regex) {
        Matcher matcher = pattern.matcher(regex);
        if (matcher.matches()) {
            return matcher;
        }
        Log.error(String.format("'%s' does not match the specified regex pattern '%s'",
                regex,
                "{serviceID = {serviceID}, instanceID = {instanceID}, eventgroupID = {eventgroupID}}"));
        return null;
    }

    protected boolean checkScope(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            return true;
        }
        Log.error(String.format("Invalid scope kind: '%s'. SomeIPConnection expects as scope kind 'NameScope'.", scope.getScopeKind()));
        return false;
    }
}
