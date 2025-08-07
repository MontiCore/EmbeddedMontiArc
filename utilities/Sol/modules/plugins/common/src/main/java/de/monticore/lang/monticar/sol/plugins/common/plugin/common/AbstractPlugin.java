/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common;

import com.google.inject.Guice;
import com.google.inject.Inject;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;

import java.util.List;

public abstract class AbstractPlugin extends AbstractMojo implements Plugin {
    @Inject private List<PluginContribution> contributions;

    @Parameter(defaultValue = "${project}", readonly = true, required = true)
    protected MavenProject mavenProject;

    /**
     * @return The MavenProject object representing the Maven project on which the plugin is executed.
     */
    public MavenProject getMavenProject() {
        return this.mavenProject;
    }

    @Override
    public final void execute() throws MojoExecutionException {
        Guice.createInjector(this.getModule()).injectMembers(this);

        try {
            this.doExecute();
        } catch(Exception exception) {
            exception.printStackTrace();
            throw new MojoExecutionException(exception.getMessage(), exception);
        }
    }

    private void doExecute() throws Exception {
        for (PluginContribution contribution : this.contributions) {
            contribution.onPluginConfigure(this);
        }

        for (PluginContribution contribution : this.contributions) {
            contribution.onPluginExecute(this);
        }

        for (PluginContribution contribution : this.contributions) {
            contribution.onPluginShutdown(this);
        }
    }
}
