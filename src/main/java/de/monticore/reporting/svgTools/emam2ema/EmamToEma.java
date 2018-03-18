package de.monticore.reporting.svgTools.emam2ema;

import de.monticore.reporting.svgTools.SVGInfo;

import java.io.IOException;
import java.util.List;

public class EmamToEma {

    public static String convertToEma(List<? extends SVGInfo> models) {
        String root = "";
        for (SVGInfo model: models) {
            root = model.getRootFile1().getAbsolutePath();
            if(model.getFileType().toUpperCase().equals("EMAM")) {
                model.setEmaToPrint(EmamToEmaPrettyPrinter.getEmaString(model.getModelFileAsString()));
            }
        }

        try {
            ModelWriter.writeModels(models, root, root + "_EMA");
        } catch (IOException e) {
            e.printStackTrace();
        }

        return root + "_EMA";
    }

}
