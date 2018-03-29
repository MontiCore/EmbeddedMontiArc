var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
define(["require", "exports", "../se.rwth.common.module/module"], function (require, exports, module_1) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class Menu extends module_1.Module {
        constructor() {
            super();
            this.$panel = jQuery("#menu-panel");
            this.$buttonFromGitHub = jQuery("#menu-item-from-github");
            this.$buttonDemo = jQuery("#menu-item-demo");
            this.$buttonCancel = jQuery("#menu-item-cancel");
        }
        static create() {
            return __awaiter(this, void 0, void 0, function* () {
            });
        }
        show() {
            this.$panel.slideDown();
            this.emit("show");
        }
        hide() {
            this.$panel.slideUp();
            this.emit("hide");
        }
        onNewProjectClick() {
        }
        onDemoClick() {
        }
        onCancelClick() {
        }
    }
    exports.Menu = Menu;
});
//# sourceMappingURL=menu.js.map