/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceKind;

import java.util.Optional;

public class EMADynamicConnectorSymbol extends EMAConnectorSymbol {

    protected boolean dynamicSourceNewComponent;
    protected boolean dynamicSourceNewPort;
    protected boolean dynamicTargetNewComponent;
    protected boolean dynamicTargetNewPort;

    public EMADynamicConnectorSymbol(String name) {
        super(name);
    }

    public EMADynamicConnectorSymbol(String name, EMAConnectorInstanceKind kind) {
        super(name, kind);
    }

    public boolean isDynamicSourceNewComponent() {
        return dynamicSourceNewComponent;
    }

    public void setDynamicSourceNewComponent(boolean dynamicSourceNewComponent) {
        this.dynamicSourceNewComponent = dynamicSourceNewComponent;
    }

    public boolean isDynamicSourceNewPort() {
        return dynamicSourceNewPort;
    }

    public void setDynamicSourceNewPort(boolean dynamicSourceNewPort) {
        this.dynamicSourceNewPort = dynamicSourceNewPort;
    }

    public boolean isDynamicTargetNewComponent() {
        return dynamicTargetNewComponent;
    }

    public void setDynamicTargetNewComponent(boolean dynamicTargetNewComponent) {
        this.dynamicTargetNewComponent = dynamicTargetNewComponent;
    }

    public boolean isDynamicTargetNewPort() {
        return dynamicTargetNewPort;
    }

    public void setDynamicTargetNewPort(boolean dynamicTargetNewPort) {
        this.dynamicTargetNewPort = dynamicTargetNewPort;
    }

    @Override
    public String getFullName() {
        return determineFullName();
    }
}
