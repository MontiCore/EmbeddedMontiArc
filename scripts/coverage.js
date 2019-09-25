/*
 * (c) https://github.com/MontiCore/monticore
 */
const FileSystem = require("fs-extra");
const Path = require("path");

async function readCoverage(indexHTML) {
    const contents = await FileSystem.readFile(indexHTML, "utf-8");
    const totalIndex = contents.indexOf("<td>Total</td>");
    const percentageIndex = contents.indexOf("%", totalIndex);
    const subContents = contents.slice(totalIndex + 14, percentageIndex + 1);
    const matches = subContents.match(/(\d+)/g);
    const percentage = matches[matches.length - 1];

    console.log(`>>> CODE COVERAGE: ${percentage}% <<<`);
}

async function execute() {
    const indexHTML = Path.join(process.cwd(), "public", "pages", "jacoco", "index.html");

    if (await FileSystem.pathExists(indexHTML)) return readCoverage(indexHTML);
    else console.warn("JaCoCo index.html could not be found.");
}

execute().catch(error => console.error(error.message));
