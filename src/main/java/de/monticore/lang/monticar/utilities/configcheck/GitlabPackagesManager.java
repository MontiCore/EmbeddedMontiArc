package de.monticore.lang.monticar.utilities.configcheck;

import org.apache.maven.model.DeploymentRepository;
import org.gitlab4j.api.GitLabApi;
import org.gitlab4j.api.GitLabApiException;
import org.gitlab4j.api.PackagesApi;
import org.gitlab4j.api.models.Package;
import java.util.ArrayList;
import java.util.List;

public class GitlabPackagesManager {
    private static final String GITLAB_API_URL = "https://git.rwth-aachen.de/";
    private static final String PRIVATE_TOKEN = "glpat-Dr2YZgwdmdPWSehyZtWq";
    private static final String PROJECT_ID = "49355";
    private final GitLabApi gitLabApi;

    public GitlabPackagesManager() {
        gitLabApi = new GitLabApi(GITLAB_API_URL, PRIVATE_TOKEN);
    }

    public List<Package> getPackages() {
        PackagesApi packagesApi = gitLabApi.getPackagesApi();
        List<Package> packages = new ArrayList<>();
        try {
            packages = packagesApi.getPackages(PROJECT_ID);
        } catch (GitLabApiException e) {
            e.printStackTrace();
        }
        return packages;
    }

    protected static DeploymentRepository getGitlabRepository() {
        DeploymentRepository deploymentRepository = new DeploymentRepository();
        deploymentRepository.setId("gitlab-maven");
        deploymentRepository.setUrl(GITLAB_API_URL + "api/v4/projects/" + PROJECT_ID + "/packages/maven");
        return deploymentRepository;
    }
}
