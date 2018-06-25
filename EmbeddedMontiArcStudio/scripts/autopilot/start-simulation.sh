pushd `pwd`
echo "TOMCAT_HOME="${TOMCAT_HOME}
cd "${TOMCAT_HOME}/bin"
exec ./catalina.sh run
popd
