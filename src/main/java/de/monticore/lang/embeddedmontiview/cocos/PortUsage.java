/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTComponentCoCo;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewConnectorSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewPortSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.stream.Collectors;

/**
 * CV5: In decomposed components all ports should be used in at least one connector.<br>
 * DIFFERENCE to CV6: CV5 checks that in and out ports are connected <em>within</em> the
 * (non-atomic) component itself while CV6 checks that a subcomponent is connected in its
 * <em>outer context</em> (i.e. the outer component).
 *
 * @author ahaber, Robert Heim
 */
public class PortUsage implements EmbeddedMontiViewASTComponentCoCo {

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

    // %%%%%%%%%%%%%%%% CV5 %%%%%%%%%%%%%%%%
    if (entry.isDecomposed()) {
      // --------- IN PORTS ----------
      // check that in-ports are used within the component
      // in->out or in->sub.in (both only occur as normal connectors where the in ports must be the
      // source)

      Collection<String> remainingPorts = getNames(entry.getIncomingPorts());

      Collection<String> connectorSources = getSourceNames(entry.getConnectors());

      if (entry.isInnerComponent()) {
        // ports not connected by the inner component itself might be connected from the parent
        // component or any of the parent's subcomponents' simple connectors
        ViewComponentSymbol componentUsingSubComp = (ViewComponentSymbol) entry.getEnclosingScope().getSpanningSymbol().get();
        connectorSources.addAll(getSourceNames(componentUsingSubComp.getConnectors()));
      }

      remainingPorts.removeAll(connectorSources);
      if (!remainingPorts.isEmpty()) {
        remainingPorts.forEach(p -> Log.error(String.format("0xAC006 Port %s is not used!", p)));
      }

      // --------- OUT PORTS ----------
      // check that out-ports are connected (i.e. they are targets of connectors)
      // they might be connected using normal connectors (in->out or sub.out->out)
      // or using simple connectors (sub.out->out) (note that simple connectors only allow the
      // subcomponents outgoing ports as source)

      remainingPorts = getNames(entry.getOutgoingPorts());
      Collection<String> connectorTargets = getTargetNames(entry.getConnectors());
      // add simple connectors of all subcomponents that might connect the ports.
      entry.getSubComponents().forEach(sc -> connectorTargets.addAll(getTargetNames(sc.getSimpleConnectors())));

      remainingPorts.removeAll(connectorTargets);
      if (!remainingPorts.isEmpty()) {
        remainingPorts.forEach(p -> Log.error(String.format("0xAC007 Port %s is not used!", p), node.get_SourcePositionStart()));
      }
    }
  }
}
