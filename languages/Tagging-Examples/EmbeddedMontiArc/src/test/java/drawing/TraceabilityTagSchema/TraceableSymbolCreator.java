/* (c) https://github.com/MontiCore/monticore */
/* generated from model null*/
/* generated by template templates.de.monticore.lang.tagschema.SimpleTagTypeCreator*/


package drawing.TraceabilityTagSchema;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
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

/**
 * created by SimpleTagTypeCreator.ftl
 */
public class TraceableSymbolCreator implements TagSymbolCreator {

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
        .filter(n -> n.endsWith("TraceabilityTagSchema"))
        .count() == 0) {
      return; // the tagging model is not conform to the TraceabilityTagSchema tagging schema
    }
    final String packageName = Joiners.DOT.join(unit.getPackage());
    final String rootCmp = // if-else does not work b/c of final (required by streams)
        (unit.getTagBody().getTargetModel().isPresent()) ?
            Joiners.DOT.join(packageName, ((ASTNameScope) unit.getTagBody().getTargetModel().get())
                .getQualifiedName().toString()) :
            packageName;

     for (ASTTag element : unit.getTagBody().getTags()) {
            element.getTagElements().stream()
              .filter(t -> t.getName().equals("Traceable"))
              .filter(t -> !t.getTagValue().isPresent()) // only marker tag with no value
              .forEachOrdered(t ->
                  element.getScopes().stream()
                    .filter(this::checkScope)
                    .map(s -> (ASTNameScope) s)
                    .map(s -> tagging.resolve(
                        Joiners.DOT.join(rootCmp, s.getQualifiedName().toString()),
                        ExpandedComponentInstanceSymbol.KIND))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .forEachOrdered(s -> tagging.addTag(s, new TraceableSymbol())));
      }
    }

  protected boolean checkScope(ASTScope scope) {
    if (scope.getScopeKind().equals("NameScope")) {
      return true;
    }
    Log.error(String.format("0xT0005 Invalid scope kind: '%s'. Traceable expects as scope kind 'NameScope'.",
        scope.getScopeKind()), scope.get_SourcePositionStart());
    return false;
  }
}
