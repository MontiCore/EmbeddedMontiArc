/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tag;//package de.monticore.lang.montiarc.tag;
//
//import java.util.Optional;
//
//import de.monticore.lang.montiarc.montiarc._symboltable.ComponentSymbol;
//import de.monticore.lang.montiarc.tagging._ast.ASTNameScope;
//import de.monticore.lang.montiarc.tagging._ast.ASTScope;
//import de.monticore.lang.montiarc.tagging._ast.ASTTag;
//import de.monticore.lang.montiarc.tagging._ast.ASTTaggingUnit;
//import de.monticore.lang.montiarc.tagging._symboltable.TagSymbolCreator;
//import de.monticore.symboltable.Scope;
//import de.se_rwth.commons.Joiners;
//import de.se_rwth.commons.logging.Log;
//
///**
// * Created by Michael von Wenckstern on 31.05.2016.
// * only for this tests --> this should be generated using the tag schema
// */
//public class IsTraceableSymbolCreator implements TagSymbolCreator {
//
//  public static Scope getGlobalScope(final Scope scope) {
//    Scope s = scope;
//    while (s.getEnclosingScope().isPresent()) {
//      s = s.getEnclosingScope().get();
//    }
//    return s;
//  }
//
//  public void create(ASTTaggingUnit unit, Scope gs) {
//    if (unit.getQualifiedNames().stream()
//        .map(q -> q.toString())
//        .filter(n -> n.endsWith("TraceabilityTagSchema"))
//        .count() == 0) {
//      return; // the tagging model is not conform to the traceability tagging schema
//    }
//    final String packageName = Joiners.DOT.join(unit.getPackage());
//    final String rootCmp = // if-else does not work b/c of final (required by streams)
//        (unit.getTagBody().getTargetModel().isPresent()) ?
//            Joiners.DOT.join(packageName, ((ASTNameScope) unit.getTagBody().getTargetModel().get())
//                .getQualifiedName().toString()) :
//            packageName;
//
//    for (ASTTag element : unit.getTagBody().getTags()) {
//          element.getTagElements().stream()
//              .filter(t -> t.getName().equals("IsTraceable"))
//              .filter(t -> !t.getTagValue().isPresent()) // only marker tag with no value
//              .forEachOrdered(t ->
//                  element.getScopes().stream()
//                    .filter(this::checkScope)
//                    .map(s -> (ASTNameScope) s)
//                    .map(s -> getGlobalScope(gs).<ComponentSymbol>resolveDown(
//                        Joiners.DOT.join(rootCmp, s.getQualifiedName().toString()),
//                        ComponentSymbol.KIND))
//                    .filter(Optional::isPresent)
//                    .map(Optional::get)
//                    .forEachOrdered(s -> s.addTag(new IsTraceableSymbol())));
//    }
//  }
//
//
//  protected boolean checkScope(ASTScope scope) {
//    if (scope.getScopeKind().equals("NameScope")) {
//      return true;
//    }
//    Log.error(String.format("0xT0005 Invalid scope kind: '%s'. IsTraceable expects as scope kind 'NameScope'.",
//        scope.getScopeKind()), scope.get_SourcePositionStart());
//    return false;
//  }
//}
