/* (c) https://github.com/MontiCore/monticore */
const FileSystem = require("fs-extra");
const Path = require("path");

async function execute() {
    await FileSystem.copy(
        Path.resolve(__dirname, "..", "build"),
        Path.resolve(__dirname, "..", "dist")
    );
}

execute().catch(error => console.error(error));
