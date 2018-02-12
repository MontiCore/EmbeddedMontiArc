package de.monticore.lang.monticar.emadl._cocos;

//check all cocos
public class EMADLCocos {


    public static EMADLCoCoChecker createPostResolveChecker(){
        return new EMADLCoCoChecker()
                .addCoCo(new CheckCNNTrainCocos())
                .addCoCo(new CheckCNNArchPostResolveCocos())
                .addCoCo(new CheckIOTypeAndDimensions());
    }

    public static EMADLCoCoChecker createPreResolveChecker(){
        return new EMADLCoCoChecker()
                .addCoCo(new CheckCNNArchPreResolveCocos())
                .addCoCo(new CheckArchitectureArgument())
                .addCoCo(new CheckConfigArgument())
                .addCoCo(new CheckArchitectureIO());
    }
}
