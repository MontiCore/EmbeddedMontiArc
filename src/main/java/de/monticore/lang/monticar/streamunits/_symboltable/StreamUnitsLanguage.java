/* (c) https://github.com/MontiCore/monticore */
/* generated from model null*/
/* generated by template symboltable.ModelingLanguage*/


package de.monticore.lang.monticar.streamunits._symboltable;


public class StreamUnitsLanguage extends StreamUnitsLanguageTOP {

    public static final String FILE_ENDING = "stream";

    public StreamUnitsLanguage() {
        super("StreamUnits Language", FILE_ENDING);
        setModelNameCalculator(new StreamUnitsModelNameCalculator());
    }

    @Override
    protected StreamUnitsModelLoader provideModelLoader() {
        return new StreamUnitsModelLoader(this);
    }

}
