define(["require", "exports", "../se.rwth.common.module/module"], function (require, exports, module_1) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class Dialog extends module_1.UIModule {
        constructor() {
            super("common.dialog");
        }
        setDependencies() { }
        setHTMLElements() {
            this.$container = jQuery("#dialog-container");
            this.$message = jQuery("#dialog-message");
            this.$icon = jQuery("#dialog-icon");
            this.$panel = jQuery("#dialog-panel");
        }
        addEventListeners() { }
        static create() {
            return Dialog.instance = Dialog.instance ? Dialog.instance : new Dialog();
        }
        show(message, type = "error", buttons = []) {
            this.removeButtons();
            this.message(message);
            this.type(type);
            this.addButtons(buttons);
            this.$container.show();
        }
        hide() {
            this.$container.hide();
        }
        message(message) {
            return message ? this.setMessage(message) : this.getMessage();
        }
        type(type) {
            return type ? this.setType(type) : this.getType();
        }
        removeButtons() {
            this.$panel.children(".button").remove();
        }
        setType(type) {
            const iconPath = Dialog.RADIX + (type === "error" ? "error.svg" : "warning.svg");
            const iconURL = "url(\"" + iconPath + "\")";
            this.$icon.css("background-image", iconURL);
        }
        getType() {
            const backgroundImage = this.$icon.css("background-image");
            const isError = backgroundImage.endsWith("error.svg");
            return isError ? "error" : "warning";
        }
        setMessage(message) {
            this.$message.html(message);
        }
        getMessage() {
            return this.$message.html();
        }
        addButtons(labels) {
            const length = labels.length;
            for (let i = 0; i < length; i++) {
                this.addButton(labels[i], i);
            }
        }
        addButton(label, index) {
            const $button = jQuery("<span/>", { "class": "button", text: label });
            const callback = () => {
                this.emit("click", index);
                this.hide();
            };
            $button.one("click", callback);
            this.$panel.append($button);
        }
    }
    Dialog.RADIX = "modules/se.rwth.common.dialog/images/";
    exports.Dialog = Dialog;
});
//# sourceMappingURL=dialog.js.map