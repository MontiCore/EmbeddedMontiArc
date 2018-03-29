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
    class Demo {
        constructor(title) {
            this.addHTMLElements();
            this.setTitle(title);
        }
        addHTMLElements() {
        }
        setTitle(title) {
            this.$title.html(title);
        }
        getTitle() {
            return this.$title.html();
        }
        getIndex() {
            return this.$item.index();
        }
    }
    exports.Demo = Demo;
    class Demos extends module_1.Module {
        constructor() {
            super();
            this.$panel = jQuery("#demos-panel");
            this.$list = jQuery("#demos-list");
        }
        static create() {
            return __awaiter(this, void 0, void 0, function* () {
            });
        }
        show() {
            this.$panel.show();
            this.emit("show");
        }
        hide() {
            this.$panel.hide();
            this.emit("hide");
        }
        load() {
            return __awaiter(this, void 0, void 0, function* () {
                const github = new GitHub();
            });
        }
        addListItems(contents) {
        }
        addListItem(content) {
        }
    }
    Demos.USERNAME = "EmbeddedMontiArc";
    Demos.REPONAME = "Demos";
    Demos.BRANCHNAME = "master";
    exports.Demos = Demos;
});
//# sourceMappingURL=demos.js.map