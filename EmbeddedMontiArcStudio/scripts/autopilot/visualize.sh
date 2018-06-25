cd ../shared
source variables.sh
cd ../intersection

java -jar $SVG_HOME/emam2ema.jar \
   $HOME/model \
   $SVG_HOME/modelEMA

rm -rf "$SVG_HOME/SVG"
mkdir "$SVG_HOME/SVG"
java -jar "$SVG_HOME/embeddedmontiarc-svggenerator.jar" \
   --input "de.rwth.armin.modeling.autopilot.autopilot" \
   --modelPath "$SVG_HOME/modelEMA/autopilot/" \
   --recursiveDrawing "true" \
   --outputPath "$SVG_HOME/SVG/"
