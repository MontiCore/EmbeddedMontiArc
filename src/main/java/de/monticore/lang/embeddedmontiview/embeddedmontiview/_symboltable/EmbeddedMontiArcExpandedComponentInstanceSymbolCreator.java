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
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 *
 * @author Michael von Wenckstern
 */
public class EmbeddedMontiArcExpandedComponentInstanceSymbolCreator {

  protected LinkedHashSet<ViewComponentSymbol> topComponents = new LinkedHashSet<>();

  public EmbeddedMontiArcExpandedComponentInstanceSymbolCreator() {
  }

  public static Scope getGlobalScope(final Scope scope) {
    Scope s = scope;
    while (s.getEnclosingScope().isPresent()) {
      s = s.getEnclosingScope().get();
    }
    return s;
  }

  protected ViewExpandedComponentInstanceBuilder createInstance(ViewComponentSymbol cmp, final Set<ResolvingFilter<? extends Symbol>> filters) {
    // TODO resolve generics and parameters
    //    System.err.println("create instance for: " + cmp.getName() + " [" + cmp.getFullName() + "]");
    ViewExpandedComponentInstanceBuilder builder = ViewExpandedComponentInstanceSymbol.builder().setSymbolReference(new ViewComponentSymbolReference(cmp.getName(), cmp.getEnclosingScope())).addPorts(cmp.getPorts()).addConnectors(cmp.getConnectors());

    // add sub components
    for (ViewComponentInstanceSymbol inst : cmp.getSubComponents()) {
      //      System.err.println("would create now: " + inst.getName() + "[" + inst.getComponentType().getFullName() + "]");
      Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");
      builder.addSubComponent(createInstance(inst.getComponentType(), filters).setName(inst.getName()).addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(), inst.getComponentType().getActualTypeArguments()).addResolvingFilters(filters).build());
      Log.debug(inst.toString(), "ComponentInstance CreateInstance PostSub");
    }

    // add inherited ports and sub components
    for (ViewComponentSymbol superCmp = cmp; superCmp.getSuperComponent().isPresent(); superCmp = superCmp.getSuperComponent().get()) {

      if (superCmp.getSuperComponent().get().getFormalTypeParameters().size() != superCmp.getSuperComponent().get().getActualTypeArguments().size()) {
        Log.error(String.format("Super component '%s' definition has %d generic parameters, but its" + "instantiation has %d binds generic parameters", superCmp.getFullName(), superCmp.getSuperComponent().get().getFormalTypeParameters().size(), superCmp.getSuperComponent().get().getActualTypeArguments().size()));
        return null;
      }

      builder.addPortsIfNameDoesNotExists(superCmp.getSuperComponent().get().getPorts(), superCmp.getSuperComponent().get().getFormalTypeParameters(), superCmp.getSuperComponent().get().getActualTypeArguments());
      builder.addConnectorsIfNameDoesNotExists(superCmp.getSuperComponent().get().getConnectors());
      Log.debug(superCmp.toString(), "superCmp pre lambda");
      superCmp.getSuperComponent().get().getSubComponents().stream().forEachOrdered(inst -> builder.addSubComponentIfNameDoesNotExists(createInstance(inst.getComponentType(), filters).setName(inst.getName()).addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(), inst.getComponentType().getActualTypeArguments()).addResolvingFilters(filters).build())

      );
      Log.debug(superCmp.toString(), "superCmp post lambda");

    }

    return builder;
  }
/*
    protected ViewExpandedComponentInstanceBuilder createInstance(ViewSymbol view, final Set<ResolvingFilter<? extends Symbol>> filters) {
        // TODO resolve generics and parameters
        //    System.err.println("create instance for: " + cmp.getName() + " [" + cmp.getFullName() + "]");
        ViewExpandedComponentInstanceBuilder builder =
                ViewExpandedComponentInstanceSymbol.builder()
                        .setSymbolReference(new ViewSymbolReference(view.getName(),
                                view.getEnclosingScope()))
                        .addConnectors(view.getConnectors());

        // add sub components
        for (ViewComponentInstanceSymbol inst : view.getSubComponents()) {
            //      System.err.println("would create now: " + inst.getName() + "[" + inst.getComponentType().getFullName() + "]");
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PreSub");
            builder.addSubComponent(
                    createInstance(inst.getComponentType(), filters)
                            .setName(inst.getName())
                            .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                                    inst.getComponentType().getActualTypeArguments()).addResolvingFilters(filters).build());
            Log.debug(inst.toString(), "ComponentInstance CreateInstance PostSub");
        }

        return builder;
    }
*/
}
