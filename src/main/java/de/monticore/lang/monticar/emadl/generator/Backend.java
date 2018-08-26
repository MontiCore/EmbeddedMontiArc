package de.monticore.lang.monticar.emadl.generator;


import de.monticore.lang.monticar.cnnarch.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNArch2MxNet;
import de.monticore.lang.monticar.cnnarch.caffe2generator.CNNArch2Caffe2;

import java.util.Optional;

public enum Backend {
    MXNET{
        @Override
        public CNNArchGenerator getGenerator() {
            return new CNNArch2MxNet();
        }
    },
    CAFFE2{
        @Override
        public CNNArchGenerator getGenerator() {
            return new CNNArch2Caffe2();
        }
    };

    public abstract CNNArchGenerator getGenerator();

    public static Optional<Backend> getBackendFromString(String backend){
        switch (backend){
            case "MXNET":
                return Optional.of(MXNET);

            case "CAFFE2":
                return Optional.of(CAFFE2);

            default:
                return Optional.empty();
        }
    }
}
