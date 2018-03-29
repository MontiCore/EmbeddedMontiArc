var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : new P(function (resolve) { resolve(result.value); }).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
define(["require", "exports", "../se.rwth.common.module/module", "../se.rwth.common.loader/loader", "../se.rwth.common.dialog/dialog"], function (require, exports, module_1, loader_1, dialog_1) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class Dashboard extends module_1.UIModule {
        constructor() {
            super("common.dashboard");
        }
        setDependencies() {
            this.loader = loader_1.Loader.create();
            this.dialog = dialog_1.Dialog.create();
        }
        setHTMLElements() {
            this.$panel = jQuery("#dashboard-panel");
            this.$list = jQuery("#dashboard-list");
            this.$itemPlus = jQuery("#dashboard-plus");
        }
        addEventListeners() {
        }
        static create() {
            return __awaiter(this, void 0, void 0, function* () {
                return Dashboard.instance = Dashboard.instance ? Dashboard.instance : new Dashboard();
            });
        }
        show() {
            this.$panel.show();
        }
        hide() {
            this.$panel.hide();
        }
        getMountPoint(index) {
            return this.mountPoints[index];
        }
        hasMountPoint(smountPoint) {
            for (let mountPoint of this.mountPoints) {
                if (mountPoint === smountPoint)
                    return true;
            }
            return false;
        }
        addMountPoint(mountPoint) {
            const mountPoints = this.mountPoints;
            const index = mountPoints.length;
            mountPoints.push(mountPoint);
            this.emit("add", mountPoint, index);
        }
        removeMountPoint(index) {
            const mountPoints = this.mountPoints;
            const mountPoint = mountPoints[index];
            mountPoints.splice(index, 1);
            this.emit("remove", mountPoint, index);
        }
        load() {
            const data = window.localStorage.getItem(Dashboard.KEY);
            this.mountPoints = JSON.parse(data);
        }
        save() {
            const data = JSON.stringify(this.mountPoints);
            window.localStorage.setItem(Dashboard.KEY, data);
        }
        addListItems() {
            for (let mountPoint of this.mountPoints) {
                this.addListItem(mountPoint);
            }
        }
        addListItem(mountPoint) {
            const parts = mountPoint.split('/');
            const $username = jQuery("<span/>", { "class": "text", "text": parts[0] || '' });
            const $reponame = jQuery("<span/>", { "class": "text", "text": parts[1] || '' });
            const $branchname = jQuery("<span/>", { "class": "text", "text": parts[2] || '' });
            const $tick = $("<span/>", { "class": "tick" });
            const $trash = $("<span/>", { "class": "trash" });
            const $item = $("<li/>").append($username, $reponame, $branchname, $tick, $trash);
            this.$list.append($item);
        }
        removeListItem(index) {
            this.$list.children().eq(index).remove();
        }
        onChecked() {
            this.loader.message("Loading Dashboard...");
            this.addListItems();
            this.loader.hide();
        }
        onTickClick(event) {
            const index = $(this).index(".tick");
            const mountPoint = this.getMountPoint(index);
            window.location.href = "ide.html?mountPoint=" + mountPoint;
        }
        onTrashbinClick() {
            const index = ;
        }
        onPlusClick() {
            this.emit("");
        }
    }
    Dashboard.KEY = "dashboard";
    exports.Dashboard = Dashboard;
});
//# sourceMappingURL=dashboard.js.map