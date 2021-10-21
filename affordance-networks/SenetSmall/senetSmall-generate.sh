echo "generating training files"
rm -rf ../../target/senetSmall
java -jar ../../bin/embedded-montiarc-math-middleware-generator-0.1.1-20210819.131454-3-jar-with-dependencies.jar senetSmall_config.json
