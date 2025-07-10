echo "Maven clean compile"
mvn clean compile -s settings.xml -U
echo "Rename directory location of dataset"
mv src/main/resources/experiments/training_data src/main/resources/experiments/data
echo "Execute AutoML pipeline"
mvn exec:java -s settings.xml -U
