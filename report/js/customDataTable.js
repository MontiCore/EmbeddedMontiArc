/* (c) https://github.com/MontiCore/monticore */
function getURLParameter(sParam) {
    var sPageURL = window.location.search.substring(1);
    var sURLVariables = sPageURL.split('&');
    for (var i = 0; i < sURLVariables.length; i++) {
        var sParameterName = sURLVariables[i].split('=');
        if (sParameterName[0] == sParam) {
            return decodeURIComponent(sParameterName[1]);
        }
    }
}

function loadJSONFiles(files, datas, callback) {
    if (files.length == 0)
        callback(datas);
    else {
        var file = files[0];
        log('Read file ' + file);
        files.shift();
        var xobj = new XMLHttpRequest();
        xobj.overrideMimeType('application/json');
        xobj.open('GET', file, true);
        var res;
        var ready = 0;
        xobj.onreadystatechange = function () {
            if (xobj.readyState == 4 && xobj.status == '200') {
                res = JSON.parse(xobj.responseText);
                datas[file] = res;
                loadJSONFiles(files, datas, callback);
            }
        };
        xobj.send(null);
    }
}

function initHeader() {
    $(tableReference + ' thead tr').clone(true).appendTo(tableReference + ' thead');
    $(tableReference + ' thead tr:eq(1)').addClass('group');
    $(tableReference + ' thead tr:eq(1) th').each(function (i) {
        $(this).text('');
    });
}

function adjustFloatingHeader(content, columnIndex) {
    var first = true;
    var second = false;
    $('.fixedHeader-floating thead tr:eq(1) th').each(function (i) {
        if (i == columnIndex) {
            $(this).html(content);
        } else {
            $(this).text('');
        }
    });
}

function initFloatingHeader(columnIndex) {
    var init = false;
    var lastIndex = -1;
    $(document).scroll(function () {
        var index = 0;
        var lastFoundIndex = -1;
        var lastFound = "";
        $(tableReference + ' tbody .group td').each(function () {

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
            if(!init) {
                $('table thead tr.group').each(function () {
                    $(this).show();
                });
            }
            lastIndex = lastFoundIndex;
            if ($('.fixedHeader-floating')[0]) {
                adjustFloatingHeader(lastFound, columnIndex);
                init = true;
            }
        }

        if (!$('.fixedHeader-floating')[0]) {
            init = false;
            $('table thead tr.group').each(function () {
                $(this).hide();
            })
        }
    });
}

function initLogMechanic(controlClass, formatFunction) {
    // Add event listener for opening and closing details
    $(tableReference + ' tbody').on('click', 'td.' + controlClass, function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);
        if (row.child.isShown()) {
            // This row is already open - close it
            row.child.hide();
            tr.removeClass('logShown');
        } else {
            // Open this row
            row.child(formatFunction(row.data())).show();
            tr.addClass('logShown');
        }
    });
}

function initGrowMechanic(growClass, shortLabel, fullLabel) {
    $('.' + growClass).on('mouseover touchstart', function () {
        $(this).find('.' + shortLabel).hide();
        $(this).find('.' + fullLabel).show();
    });
    $('.grow').on('mouseout touchend', function () {
        $(this).find('.' + shortLabel).show();
        $(this).find('.' + fullLabel).hide();
    });
}

function defaultExpand(columnToClick, columnFilterFunction) {
    var indexes = table.rows().eq(0).filter(columnFilterFunction);

    table.rows(indexes).eq(0).map(function (rowIdx) {
        table.cell(rowIdx, columnToClick).nodes().to$().click();
    });
}

expandRow = function (tr, row, data, uniqueNameFunction) {

    if (data.ChildData.length != 0) {
        var name = uniqueNameFunction(data);
        data['Order'] = name;

        if ($(tr).hasClass('childrenShown')) {
            $('.' + name + '_Child').each(function () {
                var childTr = $(this).closest('tr');
                var childRow = table.row(childTr);
                table.row(childRow).remove();
            });
            $(tr).removeClass('childrenShown');
        } else {
            for (var i = 0; i < data.ChildData.length; i++) {
                var childData = data.ChildData[i];
                childData['Order'] = name + "_Child";
                var tmpRow = table.row.add(childData).node();
                $(tmpRow).addClass('childRow1')
                $(tmpRow).addClass(name + "_Child");
            }
            $(tr).addClass('childrenShown');
        }
        table.draw();
    }

};

function childControlInit(controlClass, uniqueNameFunction) {
    $(tableReference + ' tbody').on('click', 'td.' + controlClass, function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);

        expandRow(tr, row, row.data(), uniqueNameFunction);
    });
}

function insertTimeStamp(date) {
    if (date !== "undefined") {
        $(tableReference + '_wrapper').prepend(
            '<div id="my-table_filter2" class="timeStamp">' +
            '<label>Date: ' + date + '</label>' +
            '</div>')
    }
}

var isLogging = false;
function enableLogging(value) {
    isLogging = value;
}

function log(str) {
    if (isLogging) console.log(str);
}