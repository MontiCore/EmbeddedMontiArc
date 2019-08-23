/* (c) https://github.com/MontiCore/monticore */
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testcafe_1 = require("testcafe");
const Util = require("util");
const constants_1 = require("../../se.rwth.common.test/constants");
class DialogTestController {
    constructor() {
        this.$container = testcafe_1.Selector("#dialog-container");
        this.$message = testcafe_1.Selector("#dialog-message");
        this.$icon = testcafe_1.Selector("#dialog-icon");
        this.$panel = testcafe_1.Selector("#dialog-panel");
        this.$buttons = this.$panel.child(".shared-span");
    }
    /*
     * Methods
     */
    async show(message, type, buttons, callbacks = [], error) {
        return testcafe_1.ClientFunction((message, type, buttons, callbacks, error) => Dialog.show(message, type, buttons, callbacks, error)).apply(null, arguments);
    }
    async hide() {
        return testcafe_1.ClientFunction(() => Dialog.hide()).apply(null, arguments);
    }
    /*
     * Checks
     */
    async checkVisibility(expectedVisibility, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const currentVisibility = this.$container.getStyleProperty("display");
        return testcafe_1.t.expect(currentVisibility).eql(expectedVisibility, "[Dialog]: Visibility Violation", { timeout: timeout });
    }
    async checkIcon(type, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const icon = type === "error" ? "error.svg" : "warning.svg";
        const background = await this.$icon.getStyleProperty("background-image");
        const hasIcon = background.indexOf(icon) > -1;
        return testcafe_1.t.expect(hasIcon).ok("[Dialog]: Type Violation", { timeout: timeout });
    }
    async checkMessage(message, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const span = this.$message.addCustomDOMProperties({ innerHTML: node => node.innerHTML });
        const html = await span["innerHTML"];
        return testcafe_1.t.expect(html).eql(message, "[Dialog]: Message Violation", { timeout: timeout });
    }
    async checkButtons(labels, timeout = constants_1.Constants.DEFAULT_TIMEOUT) {
        const length = labels.length;
        for (let i = 0; i < length; i++) {
            const button = this.$buttons.nth(i);
            await testcafe_1.t.expect(button.textContent).eql(labels[i], { timeout: timeout });
        }
    }
    async checkShow(message, type, labels, timeout) {
        return Promise.all([
            this.checkVisibility("block", timeout),
            this.checkMessage(message),
            this.checkIcon(type),
            this.checkButtons(labels)
        ]);
    }
    async checkHide(timeout) {
        return this.checkVisibility("none", timeout);
    }
    /*
     * Actions
     */
    async clickButton(index) {
        const button = this.$buttons.nth(index);
        const message = Util.format("[Dialog]: Button at Position %d has been clicked.", index);
        console.log(message);
        return testcafe_1.t.click(button);
    }
}
exports.DialogTestController = DialogTestController;
