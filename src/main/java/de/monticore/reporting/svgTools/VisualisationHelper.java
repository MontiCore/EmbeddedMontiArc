package de.monticore.reporting.svgTools;

import de.monticore.reporting.svgTools.emam2ema.EmamToEma;
import de.monticore.reporting.order.ChildElement;
import de.monticore.reporting.svgTools.helper.Extractor;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class VisualisationHelper {

    private final static File ICON_FOLDER = new File("report/images/icons");

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
        for (SVGInfo model : models) {
            setChildrenSVGPath(model);
        }
    }

    private static void setChildrenSVGPath(SVGInfo model) {
        if (model.getSvgPath() != null && !model.getSvgPath().equals("") && model.getChildren().size() > 0) {
            String svgPathParent = model.getSvgPath().substring(0, model.getSvgPath().lastIndexOf("html"));
            for(ChildElement childElement: model.getChildren()){
                if(((SVGInfo) childElement.getChild()).getSvgPath() == null || ((SVGInfo) childElement.getChild()).getSvgPath().equals("")) {
                    if(childElement.getChild().getChildren().size() > 0) {
                        String referencedName = childElement.getReferencedName();
                        ((SVGInfo) childElement.getChild()).setSvgPath(svgPathParent + referencedName + ".html");
                        setChildrenSVGPath((SVGInfo) childElement.getChild());
                    }
                }
            }
        }
    }
}
