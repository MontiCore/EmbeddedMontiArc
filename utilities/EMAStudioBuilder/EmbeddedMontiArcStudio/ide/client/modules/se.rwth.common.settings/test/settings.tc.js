/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
const constants_1 = require("../../se.rwth.common.test/constants");
const client_functions_1 = require("../../se.rwth.common.test/client.functions");
class SettingsTestController {
    /*
     * Methods
     */
    async set(key, value) {
        return testcafe_1.ClientFunction((key, value) => Settings.set(key, value)).apply(null, arguments);
    }
    async get(key) {
        return testcafe_1.ClientFunction((key) => Settings.get(key)).apply(null, arguments);
    }
    /*
     * Checks
     */
    async checkValue(key, expectedValue, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const data = await client_functions_1.ClientFunctions.LocalStorage.getItem(SettingsTestController.KEY) || "{}";
        const settings = JSON.parse(data);
        const actualValue = settings[key];
        return testcafe_1.t.expect(actualValue).eql(expectedValue, "[Settings]: Value Violation", { timeout: timeout });
    }
}
SettingsTestController.KEY = "settings";
exports.SettingsTestController = SettingsTestController;
