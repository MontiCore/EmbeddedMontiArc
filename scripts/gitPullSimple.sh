# clone EmbeddedMontiArc
if [ ! -d "EmbeddedMontiArc" ]
then
mkdir "EmbeddedMontiArc"
fi
cd EmbeddedMontiArc
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EmbeddedMontiArc" "EmbeddedMontiArc"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EMAM2Cpp" "EMAM2Cpp"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/Struct" "Struct"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EMA2Kotlin" "EMA2Kotlin"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/Documentation" "Documentation"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EMAM2PythonROS" "EMAM2PythonROS"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/languagescommon" "languagescommon"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/ViewVerification" "ViewVerification"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/RewriteConf" "RewriteConf"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/OCL_EMA2Java" "OCL_EMA2Java"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/OCL2Java" "OCL2Java"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/OCL" "OCL"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/NumberUnit" "NumberUnit"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EmbeddedMontiView" "EmbeddedMontiView"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EmbeddedMontiArcMath" "EmbeddedMontiArcMath"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/EmbeddedMontiArcBehaviour" "EmbeddedMontiArcBehaviour"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/visualisation" "visualisation"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/MontiMath" "MontiMath"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/reporting" "reporting"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/demonstrator" "demonstrator"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/CNNTrainLang" "CNNTrainLang"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/CNNArchLang" "CNNArchLang"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/languages" "languages"
git clone --depth 1 "https://github.com/EmbeddedMontiArc/ViewVerificationExamples" "ViewVerificationExamples"
cd ..

# clone MontiSim
if [ ! -d "MontiSim" ]
then
mkdir "MontiSim"
fi
cd MontiSim
git clone --depth 1 "https://github.com/MontiSim/CarLang" "CarLang"
git clone --depth 1 "https://github.com/MontiSim/server" "server"
git clone --depth 1 "https://github.com/MontiSim/visualization" "visualization"
git clone --depth 1 "https://github.com/MontiSim/SimLang" "SimLang"
git clone --depth 1 "https://github.com/MontiSim/simulation" "simulation"
git clone --depth 1 "https://github.com/MontiSim/commons" "commons"
git clone --depth 1 "https://github.com/MontiSim/application" "application"
git clone --depth 1 "https://github.com/MontiSim/example-autopilot-ema" "example-autopilot-ema"
git clone --depth 1 "https://github.com/MontiSim/controller" "controller"
git clone --depth 1 "https://github.com/MontiSim/Documentation" "Documentation"
cd ..