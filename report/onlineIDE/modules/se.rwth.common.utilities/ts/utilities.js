"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var Utilities = /** @class */ (function () {
    function Utilities() {
    }
    Utilities.query = function (queryString) {
        var pairs = queryString.split('&');
        var result = {};
        for (var _i = 0, pairs_1 = pairs; _i < pairs_1.length; _i++) {
            var pair = pairs_1[_i];
            var parts = pair.split('=');
            var key = parts[0];
            result[key] = parts[1];
        }
        return result;
    };
    Utilities.occurrences = function (string, search) {
        var occurrences = 0;
        var position = 0;
        while (true) {
            position = string.indexOf(search, position);
            if (position >= 0) {
                occurrences++;
                position += search.length;
            }
            else
                break;
        }
        return occurrences;
    };
    return Utilities;
}());
exports.Utilities = Utilities;
