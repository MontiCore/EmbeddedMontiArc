/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;

public abstract class AbstractTemplateContribution implements TemplateContribution {
    protected final HandCodeService handCode;

    protected AbstractTemplateContribution(HandCodeService handCode) {
        this.handCode = handCode;
    }

    @Override
    public boolean hasHandCodedPeer() {
        return this.handCode.existsHandCodedPeer(this.getOutputPath());
    }

    @Override
    public String toString() {
        return String.format("{ TemplateFile: %s }", this.getTemplatePath());
    }
}
