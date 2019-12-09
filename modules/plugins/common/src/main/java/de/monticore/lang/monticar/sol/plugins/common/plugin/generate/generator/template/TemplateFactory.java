/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import com.google.inject.assistedinject.Assisted;

public interface TemplateFactory {
    /**
     * @param templatePath The relative path to the template file.
     * @param outputPath The relative path to which the filled template should be generated to.
     * @return A configured template.
     */
    Template create(@Assisted("templatePath") String templatePath, @Assisted("outputPath") String outputPath,
                    @Assisted("arguments") Object ...arguments);

    /**
     * @param templatePath The relative path to the template file.
     * @param outputPath The relative path to which the filled template should be generated to.
     * @param suffix The suffix which will be appended to the generated file if handwritten code exists.
     * @return A configured template.
     */
    Template create(@Assisted("templatePath") String templatePath, @Assisted("outputPath") String outputPath,
                    @Assisted("suffix") String suffix, @Assisted("arguments") Object ...arguments);
}
