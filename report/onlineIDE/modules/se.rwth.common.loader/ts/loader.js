define(["require", "exports", "../se.rwth.common.module/module"], function (require, exports, module_1) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class Loader extends module_1.UIModule {
        constructor() {
            super("common.loader");
        }
        setDependencies() { }
        setHTMLElements() {
            this.$container = jQuery("#loader-container");
            this.$message = jQuery("#loader-message");
        }
        addEventListeners() { }
        static create() {
            return Loader.instance = Loader.instance ? Loader.instance : new Loader();
        }
        show(message) {
            this.$message.html(message);
            this.$container.show();
            this.emit("show", message);
        }
        hide() {
            this.$container.hide();
            this.emit("hide");
        }
        message(message) {
            this.$message.html(message);
            this.emit("message", message);
        }
    }
    exports.Loader = Loader;
});
//# sourceMappingURL=loader.js.map