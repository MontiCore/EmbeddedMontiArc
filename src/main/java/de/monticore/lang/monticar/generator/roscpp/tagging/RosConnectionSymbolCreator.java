package de.monticore.lang.monticar.generator.roscpp.tagging;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.tagging._ast.ASTNameScope;
import de.monticore.lang.tagging._ast.ASTScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class RosConnectionSymbolCreator implements TagSymbolCreator {

    /**
     * regular expression pattern:
     * topic = {({name}, {type}), msgField = {msgField}}
     * to test the pattern just enter:
     * \s*\{\s*topic\s*=\s*\(\s*([a-z|A-Z|~|/][0-9|a-z|A-Z|_|/]*)\s*,\s*([a-z|A-Z][0-9|a-z|A-Z|_|/]*)\s*\)\s*,\s*msgField\s*=\s*([a-z|A-Z][a-z|A-Z|1-9|_|\.|::|\(|\)]*)\s*\}\s*
     * at http://www.regexplanet.com/advanced/java/index.html
     */

    public static final Pattern pattern = Pattern.compile("\\s*\\{\\s*topic\\s*=\\s*\\(\\s*([a-z|A-Z|~|/][0-9|a-z|A-Z|_|/]*)\\s*,\\s*([a-z|A-Z][0-9|a-z|A-Z|_|/]*)\\s*\\)\\s*,\\s*msgField\\s*=\\s*([a-z|A-Z][a-z|A-Z|1-9|_|\\.|::|\\(|\\)]*)\\s*\\}\\s*");

    public static Scope getGlobalScope(final Scope scope) {
        Scope s = scope;
        while (s.getEnclosingScope().isPresent()) {
            s = s.getEnclosingScope().get();
        }
        return s;
    }

    public void create(ASTTaggingUnit unit, TaggingResolver tagging) {
        if (unit.getQualifiedNames().stream()
                .map(q -> q.toString())
                .filter(n -> n.endsWith("RosToEmamTagSchema"))
                .count() == 0) {
            return; // the tagging model is not conform to the traceability tagging schema
        }
        final String packageName = Joiners.DOT.join(unit.getPackage());
        final String rootCmp = // if-else does not work b/c of final (required by streams)
                (unit.getTagBody().getTargetModel().isPresent()) ?
                        Joiners.DOT.join(packageName, ((ASTNameScope) unit.getTagBody().getTargetModel().get())
                                .getQualifiedName().toString()) :
                        packageName;

        for (ASTTag element : unit.getTagBody().getTags()) {
            element.getTagElements().stream()
                    .filter(t -> t.getName().equals("RosConnection")) // after that point we can throw error messages
                    .filter(t -> t.getTagValue().isPresent())
                    .map(t -> matchRegexPattern(t.getTagValue().get()))
                    .filter(r -> r != null)
                    .forEachOrdered(m ->
                            element.getScopes().stream()
                                    .filter(this::checkScope)
                                    .map(s -> (ASTNameScope) s)
                                    .map(s -> tagging.resolve(Joiners.DOT.join(rootCmp, // resolve down does not try to reload symbol
                                            s.getQualifiedName().toString()), PortSymbol.KIND))
                                    .filter(Optional::isPresent) // if the symbol is not present, does not mean that the symbol
                                    .map(Optional::get)          // is not available at all, maybe it will be loaded later
                                    .forEachOrdered(s -> tagging.addTag(s,
                                            new RosConnectionSymbol(m.group(1), m.group(2), m.group(3)
                                            ))));
        }
    }

    protected Matcher matchRegexPattern(String regex) {
        Matcher matcher = pattern.matcher(regex);
        if (matcher.matches()) {
            return matcher;
        }
        Log.error(String.format("'%s' does not match the specified regex pattern '%s'",
                regex,
                "{topic = ({name}, {type}), msgField = {msgField}}"));
        return null;
    }

    protected boolean checkScope(ASTScope scope) {
        if (scope.getScopeKind().equals("NameScope")) {
            return true;
        }
        Log.error(String.format("Invalid scope kind: '%s'. RosConnection expects as scope kind 'NameScope'.", scope.getScopeKind()));
        return false;
    }
}