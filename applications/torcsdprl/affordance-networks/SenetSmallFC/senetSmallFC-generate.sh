echo "generating training files"
rm -rf ../../target/senetSmallFC
java -jar ../../bin/embedded-montiarc-math-middleware-generator-0.1.1-20210819.131454-3-jar-with-dependencies.jar senetSmallFC_config.json
