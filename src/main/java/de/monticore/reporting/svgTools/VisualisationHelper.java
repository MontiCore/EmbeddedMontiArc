package de.monticore.reporting.svgTools;

import de.monticore.reporting.svgTools.emam2ema.EmamToEma;
import de.monticore.reporting.order.ChildElement;
import de.monticore.reporting.svgTools.helper.Extractor;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class VisualisationHelper {

    public static void generateSVGs(List<? extends SVGInfo> allModels, List<? extends SVGInfo> rootModels, String outputPath, boolean merge) {
        Extractor.extractSVGGenerator();
        VisualisationHelperMulitThread vHelper = new VisualisationHelperMulitThread();
        vHelper.setThreadNumber(Runtime.getRuntime().availableProcessors());
        vHelper.setTimeout(60);
        File out = new File(outputPath);
        try {
            if (out.exists() && !merge)
                FileUtils.deleteDirectory(out);
            out.mkdirs();
        } catch (IOException e) {
            e.printStackTrace();
        }
        String newRoot = EmamToEma.convertToEma(allModels);
        vHelper.generateSVGs(rootModels, outputPath);
        setChildrenSVGPath(rootModels);
        File newRootFile = new File(newRoot);
        try {
            FileUtils.deleteDirectory(newRootFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void setChildrenSVGPath(List<? extends SVGInfo> models) {
        Set<SVGInfo> visited = new HashSet();
        for (SVGInfo model : models) {
            setChildrenSVGPath(model, model.getProject() + model.getModelName(), visited);
        }
    }

    private static void setChildrenSVGPath(SVGInfo model, String root, Set<SVGInfo> visited) {
        if (!visited.contains(model) && model.getSvgPath() != null && !model.getSvgPath().equals("") && model.getChildren().size() > 0) {
            String svgPathParent = model.getSvgPath().substring(0, model.getSvgPath().lastIndexOf("html"));
            for(ChildElement childElement: model.getChildren()){
                if(((SVGInfo) childElement.getChild()).getSvgPath() == null || ((SVGInfo) childElement.getChild()).getSvgPath().equals("")) {
                    if(childElement.getChild().getChildren().size() > 0) {
                        String referencedName = childElement.getReferencedName();
                        ((SVGInfo) childElement.getChild()).setSvgPath(svgPathParent + referencedName + ".html");
                    }
                    setChildrenSVGPath((SVGInfo) childElement.getChild(), root, visited);
                }
            }
        } else if(!visited.contains(model)){
            if ((model.getSvgPath() == null || model.getSvgPath().equals("")) &&
                    !(model.getProject() + model.getModelName()).equals(root)){
                model.addErrorMessage("[INFO] do SVG generation<br>=========================");
                model.addErrorMessage("[INFO] See at " + root);
            }
            if (model.getChildren().size() > 0) {
                for(ChildElement childElement: model.getChildren())
                    setChildrenSVGPath((SVGInfo) childElement.getChild(), root, visited);
            }
        }
        visited.add(model);
    }
}
