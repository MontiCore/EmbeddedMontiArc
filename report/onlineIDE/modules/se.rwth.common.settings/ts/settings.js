define(["require", "exports"], function (require, exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class Settings {
        constructor() {
            this.doLoad();
        }
        static getInstance() {
            return Settings.instance ? Settings.instance : (Settings.instance = new Settings);
        }
        static load() {
            return Settings.getInstance().doLoad();
        }
        doLoad() {
            const data = window.localStorage.getItem(Settings.KEY) || "{}";
            this.settings = JSON.parse(data);
        }
        static save() {
            Settings.getInstance().doSave();
        }
        doSave() {
            const data = JSON.stringify(this.settings);
            window.localStorage.setItem(Settings.KEY, data);
        }
        static set(key, value) {
            Settings.getInstance().doSet(key, value);
        }
        doSet(key, value) {
            this.settings[key] = value;
            this.doSave();
        }
        static get(key) {
            return Settings.getInstance().doGet(key);
        }
        doGet(key) {
            return this.settings[key];
        }
    }
    Settings.KEY = "settings";
    exports.Settings = Settings;
});
//# sourceMappingURL=Settings.js.map