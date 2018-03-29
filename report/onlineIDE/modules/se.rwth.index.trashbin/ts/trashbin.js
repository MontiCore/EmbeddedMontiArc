"use strict";
var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
var module_1 = require("../se.rwth.common.framework/module");
var Trashbin = /** @class */ (function (_super) {
    __extends(Trashbin, _super);
    function Trashbin() {
        var _this = _super.call(this) || this;
        _this.load();
        return _this;
    }
    Trashbin.create = function () {
        return Trashbin.instance ? Trashbin.instance : (Trashbin.instance = new Trashbin);
    };
    Trashbin.prototype.load = function () {
        var data = window.localStorage.getItem(Trashbin.KEY) || "[]";
        this.mountPoints = JSON.parse(data);
        this.emit("load");
    };
    Trashbin.prototype.save = function () {
        var data = JSON.stringify(this.mountPoints);
        window.localStorage.setItem(Trashbin.KEY, data);
        this.emit("save");
    };
    Trashbin.prototype.contains = function (mountPoint) {
        return this.mountPoints.indexOf(mountPoint) > -1;
    };
    Trashbin.prototype.add = function (mountPoint) {
        this.mountPoints.push(mountPoint);
        this.emit("add", mountPoint);
        this.save();
    };
    Trashbin.prototype.remove = function (mountPoint) {
        var index = this.mountPoints.indexOf(mountPoint);
        this.mountPoints.splice(index, 1);
        this.emit("remove", mountPoint);
        this.save();
    };
    Trashbin.prototype.empty = function () {
        var _this = this;
        var onSuccess = function () {
            var mountPoint = _this.mountPoints.shift();
            if (mountPoint)
                _this.handleEmpty(mountPoint, onSuccess);
            else
                _this.save();
        };
        onSuccess();
    };
    Trashbin.prototype.handleEmpty = function (mountPoint, onSuccess) {
        var request = window.indexedDB.deleteDatabase(mountPoint);
        function onError(event) {
            console.error(event.result.target);
            this.save();
        }
        request.onsuccess = onSuccess;
        request.onerror = onError;
    };
    Trashbin.KEY = "trashbin";
    return Trashbin;
}(module_1.Module));
exports.Trashbin = Trashbin;
