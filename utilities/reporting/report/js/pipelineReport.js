/* (c) https://github.com/MontiCore/monticore */
function createTable_custom(file) {
    loadJSON(file, computeDataThenCreateTable);
}

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

function computeDataThenCreateTable(data) {
    data.forEach(function (datum) {
        var order = uniqueNameFunction_custom(datum);
        datum['Order'] = order;
    });
    createTable(data);
}

function createTable(data) {
    table = $(tableReference).DataTable({
        "fnInitComplete": function () {
            init();
        },
        "fixedHeader": true,
        "select": true,
        "data": data,
        "orderCellsTop": true,
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {"pre": [0, 'asc']},
        "aoColumns": [
            {"data": "project", "visible": false},
            {"data": "Order", "visible": false},
            {
                "className": "expandChildren-control",
                "data": "name",
                "bSortable": true,
            },
            {"data": "status", sort: "string"}
        ]
    });
};

function init() {
    table = $(tableReference).DataTable();
    childControlInit('expandChildren-control', uniqueNameFunction_custom);
}

function uniqueNameFunction_custom(data) {
    var postFix = data['branch'];
    if (postFix == "main")
        postFix = "0000000";
    else if (postFix == "master")
        postFix = "0000001";
    var res = data['project'] + "_" + data['branch'];
    res = res.split(' ').join('');
    res = res.replace(/\./g, "_").replace(/\(/g, "").replace(/\)/g, "")
        .replace(/\\/g, "_").replace(/\//g, "_");
    return res;
}
