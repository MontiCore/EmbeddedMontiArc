rm -rf ${CPP_DIR}
# (c) https://github.com/MontiCore/monticore  
mkdir ${CPP_DIR}
${JAVA_HOME}/bin/java -jar "${HOME}/emam2cpp.jar" \
   --models-dir="${HOME}/model/autopilot" \
   --root-model=de.rwth.armin.modeling.autopilot.autopilot \
   --output-dir="${CPP_DIR}" \
   --flag-generate-autopilot-adapter \
   --flag-use-armadillo-backend
cp -r ${HOME}/precompiled ${CPP_DIR}
