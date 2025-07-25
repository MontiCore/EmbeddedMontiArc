/* (c) https://github.com/MontiCore/monticore */
function loadJSON(file, callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    var res;
    var ready = 0;
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            res = JSON.parse(xobj.responseText);
            callback(res);
        }
    };
    xobj.send(null);
}

function createTable(data) {
    table = $(tableReference).DataTable({
        "fnInitComplete": function() { init(); },
        "fixedHeader": true, // needed for floating header
        "select": true, // visible selection of rows
        "data": data,
        "paging": false,
        "bInfo": false,
        "orderFixed": { "pre": [0, "asc"] }, // for the group to stick together
        "rowGroup": { // custom group information
            "dataSrc": "Root",
            "startRender": function (rows, group) {
                var count = rows.count();
                return '<span class="nowrap">' + group + ' (' + count + ')</span>';
            }
        },
        "aoColumns": [
            { "data": "Root", "visible": false },
            { "data": "DataForColumn1", "bSortable": true },
            { "data": "DataForColumn2", "bSortable": true },
            { "data": "DataForColumn3", "bSortable": true },
        ]
    });
};

function init() {
    table = $(tableReference).DataTable();
    initFloatingHeader(0);
}
