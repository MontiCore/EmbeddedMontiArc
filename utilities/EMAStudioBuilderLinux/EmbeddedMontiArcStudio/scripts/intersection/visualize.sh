cd ../shared
# (c) https://github.com/MontiCore/monticore  
source variables.sh
cd ../intersection

java -jar $SVG_HOME/emam2ema.jar \
   $HOME/model \
   $SVG_HOME/modelEMA

rm -rf "$SVG_HOME/SVG"
mkdir "$SVG_HOME/SVG"
java -jar "$SVG_HOME/embeddedmontiarc-svggenerator.jar" \
   --input "ba.system" \
   --modelPath "$SVG_HOME/modelEMA/intersection/" \
   --recursiveDrawing "true" \
   --outputPath "$SVG_HOME/SVG/"
