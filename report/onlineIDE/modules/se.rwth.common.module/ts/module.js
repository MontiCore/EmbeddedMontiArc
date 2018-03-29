define(["require", "exports"], function (require, exports) {
    "use strict";
    Object.defineProperty(exports, "__esModule", { value: true });
    class EventBus {
        constructor() {
            this.events = {};
        }
        on(eventName, callback) {
            jQuery(this.events).on(eventName, callback);
        }
        off(eventName, callback) {
            jQuery(this.events).off(eventName, callback);
        }
        once(eventName, callback) {
            jQuery(this.events).one(eventName, callback);
        }
        emit(eventName, ...args) {
            jQuery(this.events).trigger(eventName, args);
        }
        static on(id, event, callback) {
            const module = Module.get(id);
            if (module)
                module.on(event, callback);
        }
        static once(id, event, callback) {
            const module = Module.get(id);
            if (module)
                module.once(event, callback);
        }
        static off(id, event, callback) {
            const module = Module.get(id);
            if (module)
                module.off(event, callback);
        }
    }
    exports.EventBus = EventBus;
    class Module extends EventBus {
        constructor(id) {
            super();
            this.setProperties(id);
            this.setDependencies();
        }
        setProperties(id) {
            this.id = id;
        }
        load() {
            Module.add(this);
            this.emit("load");
        }
        unload() {
            Module.remove(this);
            this.emit("unload");
        }
        static add(module) {
            Module.list[module.id] = module;
        }
        static remove(module) {
            delete Module.list[module.id];
        }
        static get(id) {
            return Module.list[id];
        }
    }
    Module.list = {};
    exports.Module = Module;
    class UIModule extends Module {
        constructor(id) {
            super(id);
            this.setHTMLElements();
            this.addEventListeners();
        }
    }
    exports.UIModule = UIModule;
});
//# sourceMappingURL=module.js.map