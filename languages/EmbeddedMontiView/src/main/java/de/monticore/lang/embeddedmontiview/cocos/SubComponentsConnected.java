/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewConnectorSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewPortSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

/**
 * CV6: All ports of subcomponents should be used in at least one connector.<br>
 * DIFFERENCE to CV5: CV5 checks that in and out ports are connected <em>within</em> the
 * (non-atomic) component itself while CV6 checks that a subcomponent is connected in its
 * <em>outer context</em> (i.e. the outer component).
 *
 */
public class SubComponentsConnected
    implements EmbeddedMontiViewASTComponentCoCo {

  private Collection<String> getNames(Collection<ViewPortSymbol> ports) {
    return ports.stream().map(p -> p.getName()).collect(Collectors.toList());
  }

  private Collection<String> getSourceNames(Collection<ViewConnectorSymbol> connectors) {
    return connectors.stream().map(c -> c.getSource()).collect(Collectors.toList());
  }

  private Collection<String> getTargetNames(Collection<ViewConnectorSymbol> connectors) {
    return connectors.stream().map(c -> c.getTarget()).collect(Collectors.toList());
  }

  @Override
  public void check(ASTComponent node) {
    ViewComponentSymbol entry = (ViewComponentSymbol) node.getSymbol().get();
    // Implemented on the symTab as it takes auto-instantiation into account which is not reflected
    // in the AST.
    for (ViewComponentInstanceSymbol sub : entry.getSubComponents()) {
      // ------- IN PORTS -------
      // in ports must be connected
      // outer.in->sub.in
      // outer.AnySub.out->sub.in
      // connectors with sub.in as target occur in outer.connectors or as
      // outer.AnySub.simpleconnectors

      Collection<String> remainingSubIn = getNames(sub.getComponentType().getIncomingPorts());
      // Connectors in the outer context always refer to the ports in a relative-qualified way (e.g.
      // sub.portX) and hence we must prefix the remaining ones with sub's name to compare sets of
      // relative-qualified names
      remainingSubIn = remainingSubIn.stream().map(s -> sub.getName() + "." + s).collect(Collectors.toList());

      Collection<String> outerConnectorTargets = getTargetNames(entry.getConnectors());
      remainingSubIn.removeAll(outerConnectorTargets);
      if (!remainingSubIn.isEmpty()) {
        Collection<String> outerSubSimpleConnectorTargets = getTargetNames(entry.getSubComponents().stream().flatMap(sc -> sc.getSimpleConnectors().stream()).collect(Collectors.toList()));
        remainingSubIn.removeAll(outerSubSimpleConnectorTargets);
        if (!remainingSubIn.isEmpty()) {
          remainingSubIn.forEach(p -> Log.error(String.format("0xAC008 Port %s of subcomponent %s is not used!", p, sub.getFullName()), node.get_SourcePositionStart()));
        }
      }
      // ------- OUT PORTS -------
      // sub.out->outer.out
      // sub.out->outer.AnySub.in
      // connectors with sub.out as source occur as outer.connectors or
      // outer.AnySub.simpleConnectors
      Collection<String> remainingSubOut = getNames(sub.getComponentType().getOutgoingPorts());
      // Connectors in the outer context always refer to the ports in a relative-qualified way (e.g.
      // sub.portX) and hence we must prefix the remaining ones with sub's name to compare sets of
      // relative-qualified names
      remainingSubOut = remainingSubOut.stream().map(s -> sub.getName() + "." + s).collect(Collectors.toList());

      Collection<String> outerConnectorSources = getSourceNames(entry.getConnectors());
      remainingSubOut.removeAll(outerConnectorSources);

      if (!remainingSubOut.isEmpty()) {
        // qualified sources of simple connectors
        List<Object> outerSubSimpleConnectorSources = entry.getSubComponents().stream().flatMap(sc -> sc.getSimpleConnectors().stream()
            // map connector to qualified source name
            .map(c -> sc.getName() + "." + c.getSource())).collect(Collectors.toList());
        remainingSubOut.removeAll(outerSubSimpleConnectorSources);
        if (!remainingSubOut.isEmpty()) {
          remainingSubOut.forEach(p -> Log.error(String.format("0xAC009 Port %s of subcomponent %s is not used!", p, sub.getFullName()), node.get_SourcePositionStart()));
        }
      }
    }
  }
}
