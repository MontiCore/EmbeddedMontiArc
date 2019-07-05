/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.template.theia.common;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc.HandCodeService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.AbstractTemplateContribution;

import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class IndexTemplate extends AbstractTemplateContribution {
    @Inject
    protected IndexTemplate(HandCodeService handCode) {
        super(handCode);
    }

    @Override
    public String getTemplatePath() {
        return "templates/language-client/theia/src/common/index.ftl";
    }

    @Override
    public Path getOutputPath() {
        return Paths.get("common/index.ts");
    }
}
