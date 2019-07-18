package de.monticore.lang.monticar.emadl.generator;


import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2Gluon;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNTrain2Gluon;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNArch2MxNet;
import de.monticore.lang.monticar.cnnarch.caffe2generator.CNNArch2Caffe2;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNTrain2MxNet;
import de.monticore.lang.monticar.cnnarch.caffe2generator.CNNTrain2Caffe2;
import de.monticore.lang.monticar.emadl.generator.reinforcementlearning.RewardFunctionCppGenerator;

import java.util.Optional;

public enum Backend {
    MXNET{
        @Override
        public CNNArchGenerator getCNNArchGenerator() {
            return new CNNArch2MxNet();
        }
        @Override
        public CNNTrainGenerator getCNNTrainGenerator() {
            return new CNNTrain2MxNet();
        }
    },
    CAFFE2{
        @Override
        public CNNArchGenerator getCNNArchGenerator() {
            return new CNNArch2Caffe2();
        }
        @Override
        public CNNTrainGenerator getCNNTrainGenerator() {
            return new CNNTrain2Caffe2();
        }
    },
    GLUON{
        @Override
        public CNNArchGenerator getCNNArchGenerator() {
            return new CNNArch2Gluon();
        }
        @Override
        public CNNTrainGenerator getCNNTrainGenerator() {
            return new CNNTrain2Gluon(new RewardFunctionCppGenerator());
        }
    };

    public abstract CNNArchGenerator getCNNArchGenerator();
    public abstract CNNTrainGenerator getCNNTrainGenerator();

    public static Optional<Backend> getBackendFromString(String backend){
        switch (backend){
            case "MXNET":
                return Optional.of(MXNET);

            case "CAFFE2":
                return Optional.of(CAFFE2);

            case "GLUON":
                return Optional.of(GLUON);

            default:
                return Optional.empty();
        }
    }

    public static String getBackendString(Backend backend){
        switch (backend){
            case CAFFE2:
                return "CAFFE2";
            case GLUON:
                return "GLUON";
            default:
                return "MXNET";
        }
    }
}