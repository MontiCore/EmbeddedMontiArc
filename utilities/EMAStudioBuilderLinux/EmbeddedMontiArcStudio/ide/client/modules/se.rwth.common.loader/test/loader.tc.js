/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
const constants_1 = require("../../se.rwth.common.test/constants");
class LoaderTestController {
    constructor() {
        this.$container = testcafe_1.Selector("#loader-container");
        this.$message = testcafe_1.Selector("#loader-message");
    }
    /*
     * Methods
     */
    async show(message) {
        return testcafe_1.ClientFunction((message) => Loader.show(message)).apply(null, arguments);
    }
    async hide() {
        return testcafe_1.ClientFunction(() => Loader.hide()).apply(null, arguments);
    }
    async message(message) {
        return testcafe_1.ClientFunction((message) => Loader.message(message)).apply(null, arguments);
    }
    /*
     * Checks
     */
    async checkVisibility(expectedVisibility, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const currentVisibility = this.$container.getStyleProperty("display");
        return testcafe_1.t.expect(currentVisibility).eql(expectedVisibility, "[Loader]: Visibility Violation", { timeout: timeout });
    }
    async checkShow(message, timeout) {
        return Promise.all([
            this.checkVisibility("block", timeout),
            this.checkMessage(message)
        ]);
    }
    async checkHide(timeout) {
        return this.checkVisibility("none", timeout);
    }
    async checkMessage(message, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        return testcafe_1.t.expect(this.$message.innerText).eql(message, "[Loader]: Message Violation", { timeout: timeout });
    }
}
exports.LoaderTestController = LoaderTestController;
