/*
 * (c) https://github.com/MontiCore/monticore
 */
import { theia } from "./theia";

import * as minimist from "minimist";

const argv = minimist(process.argv.slice(2));

async function execute() {
    if (argv._[0] === "theia") return theia(argv);
}

execute().catch(error => console.error(error.message));
