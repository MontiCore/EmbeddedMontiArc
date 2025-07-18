#echo "Maven clean compile"
#mvn clean compile -s settings.xml -U
#echo "Rename directory location of dataset"
#mv src/main/resources/experiments/training_data src/main/resources/experiments/data
echo "Execute AutoML pipeline"
#mvn exec:java -s settings.xml -U -e

#needed only locally
export JAVA_HOME=/usr/lib/jvm/java-1.8.0-openjdk-amd64
export Torch_DIR=/home/antonio/.local/lib/python3.8/site-packages
mvn clean install -U -s settings.xml -e -DskipTests

mvn dependency:resolve emadl:train -s settings.xml -U -e -DskipTests