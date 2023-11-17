package de.monticore.lang.monticar.utilities.configcheck;

import javax.ws.rs.DefaultValue;

public class ConfigCheck {
    private String gitlabPackagesUrl;
    private String mlflowTrackingUrl;
    private String pathTmp;

    public String getGitlabPackagesUrl() {
        return gitlabPackagesUrl;
    }

    public String getMlflowTrackingUrl() {
        return mlflowTrackingUrl;
    }

    public String getPathTmp() {
        return pathTmp != null ? pathTmp : "target/tmp/config-check";
    }
}
