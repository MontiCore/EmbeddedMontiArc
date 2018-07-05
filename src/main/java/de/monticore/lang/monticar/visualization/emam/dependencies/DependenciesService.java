package de.monticore.lang.monticar.visualization.emam.dependencies;

public interface DependenciesService {
    void download() throws Exception;
    void install() throws Exception;
    void clean() throws Exception;
}
