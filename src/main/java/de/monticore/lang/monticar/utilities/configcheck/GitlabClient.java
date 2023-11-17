package de.monticore.lang.monticar.utilities.configcheck;

import org.gitlab4j.api.GitLabApi;

import java.util.List;

public class GitlabClient {
    private GitLabApi gitLabApi;

    public GitlabClient(String gitlabHost, String accessToken) {
        this.gitLabApi = new GitLabApi(gitlabHost, accessToken);
    }

//    public List getPackages(String filter) {
//        this.gitLabApi.
//    }
}
