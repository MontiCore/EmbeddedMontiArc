
pushd %~dp0
call mvn clean install -s "settings.xml"
cd target
ren "basic-simulator-*.jar" "basic-simulator.jar"
copy "basic-simulator.jar" "..\install"
popd
