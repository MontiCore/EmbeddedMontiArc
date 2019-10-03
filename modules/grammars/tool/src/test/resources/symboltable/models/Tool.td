package models;

tool Tool {
    attribute alias = TOOL in ENV;
    attribute path = "some/imaginary/path";
    attribute prefix = "java -jar";
    attribute environment = ToolEnvironment;

    resource SOME_RESOURCE {
        attribute alias = RESOURCE in ENV;
        attribute path = "some/imaginary/path";
    }
}