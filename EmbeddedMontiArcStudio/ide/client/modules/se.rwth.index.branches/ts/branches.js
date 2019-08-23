/* (c) https://github.com/MontiCore/monticore */
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
var Branch = /** @class */ (function () {
    function Branch() {
    }
    return Branch;
}());
exports.Branch = Branch;
var Branches = /** @class */ (function (_super) {
    __extends(Branches, _super);
    function Branches() {
        var _this = _super.call(this) || this;
        _this.$panel = jQuery("#branches-panel");
        _this.$list = jQuery("#branches-list");
        _this.$itemBack = jQuery("#branches-back");
        return _this;
    }
    Branches.prototype.show = function () {
        this.$panel.show();
        this.emit("show");
    };
    Branches.prototype.hide = function () {
        this.$panel.hide();
        this.emit("hide");
    };
    return Branches;
}(module_1.Module));
exports.Branches = Branches;
