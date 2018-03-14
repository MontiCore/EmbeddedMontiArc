function loadJSON(file, callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            callback(xobj.responseText);
        }
    };
    xobj.send(null);
}

function createTable(data) {
    data = JSON.parse(data);
    table = $('#my-table').DataTable({
        fixedHeader: true,
        "data": data,
        "language": {
            "loadingRecords": "Please wait - loading..."
        },
        "processing": true,
        "orderCellsTop": true,
        "aaSorting": [1, 'asc'],
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {
            "pre": [0, 'asc']
        },
        "select": true,
        "rowGroup": {
            dataSrc: "Root",
            startRender: function (rows, group) {
                return group + ' (' + rows.count() + ')';
            },
            endRender: null //function( rows, group ) {return group;}
        },
        "bAutoWidth": false,
        "aoColumns": [
            {"data": "Root", visible: false},
            {"data": "Order", visible: false},
            {"data": "Name", sort: "string", type: "alt-string"},
            {
                "data": "OnlineIDE",
                "className": "onlineIDE",
                "orderable": false
            }
        ],
        "order": [[1, 'asc']]
    });
};

function adjustHeader() {
    $('#my-table thead tr:eq(1) th').each(function (i) {
        $(this).text('');
    });
}

function adjustFloatingHeader(content) {
    var first = true;
    var second = false;
    $('.fixedHeader-floating thead tr:eq(1) th').each(function (i) {
        if (first) {
            first = false;
            second = true;
        } else if (second) {
            second = false;
            $(this).html(content);
        } else {
            $(this).text('');
        }
    });
}

$(document).ready(function () {

    $('#my-table thead tr').clone(true).appendTo('#my-table thead');
    $('#my-table thead tr:eq(1)').addClass('group');
    adjustHeader();

    loadJSON("data/dataGrammars.json", createTable);

    var init = false;
    var lastIndex = -1;
    $(document).scroll(function () {
        var currentGroup = "EmbeddedMontiArc";
        var index = 0;
        var lastFoundIndex = -1;
        var lastFound = "";
        $('#my-table tbody .group td').each(function () {

            var docViewTop = $(window).scrollTop();
            var elemTop = $(this).offset().top;
            var elemBottom = elemTop + $(this).height();

            if (elemBottom < docViewTop || elemTop < docViewTop) {
                lastFound = $(this).html();
                lastFoundIndex = index;
            }
            index++;
        });

        if (lastFoundIndex != lastIndex || init == false) {
            lastIndex = lastFoundIndex;
            if ($('.fixedHeader-floating')[0]) {
                adjustFloatingHeader(lastFound);
                init = true;
            }
        }

        if (!$('.fixedHeader-floating')[0]) {
            init = false;
        }
    });
});
