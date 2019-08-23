/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
class BrowserDetectTestController {
    /*
     * Methods
     */
    async browser() {
        return testcafe_1.ClientFunction(() => browser()).apply(null, arguments);
    }
}
exports.BrowserDetectTestController = BrowserDetectTestController;
