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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;

import java.util.Iterator;
import java.util.Optional;

/**
 * Symboltable entry for connectors.
 *
 * @author Arne Haber, Michael von Wenckstern
 */
public class EMAConnectorInstanceSymbol extends EMAConnectorSymbol implements EMAElementInstanceSymbol {

    /* generated by template symboltable.symbols.KindConstantDeclaration */

    public static final EMAConnectorInstanceKind KIND = EMAConnectorInstanceKind.INSTANCE;

    /**
     * Source of this connector.
     */
    protected String source;

    /**
     * Target of this connector.
     */
    protected String target;

    protected boolean isConstant = false;

    /**
     * is null if not a constantConnector
     */
    protected EMAConstantPortSymbol emaConstantPortSymbol = null;

    /**
     * use {@link #builder()}
     */
    protected EMAConnectorInstanceSymbol(String name) {
        super(TypesPrinter.fixTargetName(name), KIND);
    }

    public static EMAConnectorBuilder builder() {
        return new EMAConnectorBuilder();
    }

    public void setIsConstantConnector(boolean isConstantConnector) {
        this.isConstant = isConstantConnector;
    }

    public boolean isConstant() {
        return isConstant;
    }

    public void setEMAConstantPortSymbol(EMAConstantPortSymbol portSymbol) {
        this.emaConstantPortSymbol = portSymbol;
        setIsConstantConnector(true);
    }

    /**
     * NOTE: This method is not supported for ConstantConnectors
     *
     * @return the source
     */
    public String getSource() {

        return source;
    }

    /**
     * @param source the source to set
     */
    public void setSource(String source) {
        this.source = source;
    }

    protected EMAPortInstanceSymbol getPort(String name) {
        if (this.getEnclosingScope() == null) {
            Log.warn("ConnectorInstance does not belong to a componentInstance, cannot resolve port");
            return null;
        }
        if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
            Log.warn(
                    "ConnectorInstance is not embedded in an expanded component instance symbol, cannot resolve port");
            return null;
        }

        // (1) try to load EMAComponentInstance.Port
        String fullSource = Joiners.DOT.join(this.getPackageName(), this.getEnclosingScope().getSpanningSymbol().get().getName(), name);
        Optional<EMAPortInstanceSymbol> port = this.getEnclosingScope().<EMAPortInstanceSymbol>resolve(fullSource,
                EMAPortInstanceSymbol.KIND);
        if (port.isPresent()) {
            return port.get();
        }

        // (2) try to load EMAComponentInstance.instance.Port
        Iterator<String> parts = Splitters.DOT.split(name).iterator();
        Log.debug("" + name, "NAME:");
        if (!parts.hasNext()) {
            Log.warn("name of connector's source/target is empty, cannot resolve port");
            return null;
        }
        String instance = parts.next();
        Log.debug("" + instance, "instance");
        if (!parts.hasNext()) {
            Log.warn(
                    "name of connector's source/target does has two parts: instance.port, cannot resolve port");
            return null;
        }
        String instancePort = parts.next();
        Log.debug("" + instancePort, "instancePort");
        Optional<EMAComponentInstanceSymbol> inst = this.getEnclosingScope().getSpanningSymbol().get().getSpannedScope().
                <EMAComponentInstanceSymbol>resolve(instance, EMAComponentInstanceSymbol.KIND);
        if (!inst.isPresent()) {
            Log.warn(String.format("Could not find instance %s in connector scope, cannot resolve port",
                    instance));
            return null;
        }
        port = inst.get().getSpannedScope()
                .resolve(instancePort, EMAPortInstanceSymbol.KIND);

        if (port.isPresent()) {
            return port.get();
        }
        Log.debug("No case match for" + name, "cannot resolve port");
        return null;
    }

    /**
     * does not return Optional, since every connector has a port if the model is well-formed
     */
    public EMAPortInstanceSymbol getSourcePort() {
        return getPort(this.getSource());
    }

    /**
     * does not return Optional, since every connector has a port if the model is well-formed
     */
    public EMAPortInstanceSymbol getTargetPort() {
        return getPort(this.getTarget());
    }

    /**
     * returns the component which defines the connector this is independent from the component to
     * which the source and target ports belong to

     */
    public Optional<ComponentSymbol> getComponent() {
        if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
            return Optional.empty();
        }

        return Optional.of(((EMAComponentInstanceSymbol) this.getEnclosingScope().getSpanningSymbol().get()).getComponentType().getReferencedSymbol());
    }

    /**
     * returns the component instance which defines the connector
     */
    public Optional<EMAComponentInstanceSymbol> getComponentInstance() {
        if (!this.getEnclosingScope().getSpanningSymbol().isPresent()) {
            return Optional.empty();
        }
        return Optional.of((EMAComponentInstanceSymbol) this.getEnclosingScope().getSpanningSymbol().get());
    }

    /**
     * @return the target
     */
    public String getTarget() {
        return target;
    }

    /**
     * @param target the target to set
     */
    public void setTarget(String target) {

        this.target = TypesPrinter.fixTargetName(target);
    }

    @Override
    public String toString() {
        return SymbolPrinter.printConnector(this);
    }

    @Override
    public String getName() {
        return getTarget();
    }


}
