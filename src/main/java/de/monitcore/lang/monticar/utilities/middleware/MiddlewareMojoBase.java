package de.monitcore.lang.monticar.utilities.middleware;

import de.monitcore.lang.monticar.utilities.StreamTestMojoBase;
import org.apache.maven.plugins.annotations.Parameter;

import java.util.ArrayList;
import java.util.List;

public class MiddlewareMojoBase extends StreamTestMojoBase {

    //<editor-fold desc="Parameter">
    @Parameter(defaultValue = "roscpp")
    protected List<MiddlewareGenerator> middlewareGenerator;
    public List<MiddlewareGenerator> getMiddlewareGenerator() {
        return middlewareGenerator;
    }
    public void setMiddlewareGenerator(List<MiddlewareGenerator> middlewareGenerator) {
        this.middlewareGenerator = middlewareGenerator;
    }
    public void addMiddlewareGenerator(MiddlewareGenerator mg){
        if(middlewareGenerator == null){
            middlewareGenerator = new ArrayList<>();
        }

        if(!middlewareGenerator.contains(mg)){
            middlewareGenerator.add(mg);
        }
    }



    @Parameter
    protected List<String> middlewareRootModels;
    public List<String> getMiddlewareRootModels() {
        return middlewareRootModels;
    }
    public void setMiddlewareRootModels(List<String> middlewareRootModels) {
        this.middlewareRootModels = middlewareRootModels;
    }
    public void addMiddlewareRootModel(String item){
        if(middlewareRootModels == null){
            middlewareRootModels = new ArrayList<>();
        }
        middlewareRootModels.add(item);
    }

    @Parameter(defaultValue = "./target/middleware")
    protected String pathMiddlewareOut;
    public String getPathMiddlewareOut() {
        return pathMiddlewareOut;
    }
    public void setPathMiddlewareOut(String pathMiddlewareOut) {
        this.pathMiddlewareOut = pathMiddlewareOut;
    }



    @Parameter(defaultValue = "true")
    protected boolean runStreamTestBefore;
    public boolean getRunStreamTestBefore() {
        return runStreamTestBefore;
    }
    public void setRunStreamTestBefore(boolean runStreamTestBefore) {
        this.runStreamTestBefore = runStreamTestBefore;
    }
    //</editor-fold>


    protected void copyPropertiesAndParametersTo(MiddlewareMojoBase mmb) {
        super.copyPropertiesAndParametersTo((StreamTestMojoBase)mmb);

        mmb.middlewareGenerator = this.middlewareGenerator;
        mmb.middlewareRootModels = this.middlewareRootModels;
        mmb.pathMiddlewareOut = this.pathMiddlewareOut;
        mmb.runStreamTestBefore = this.runStreamTestBefore;
    }
}
