/**
 * Created by vrushali on 25/08/15.
 */
require(['common'], function (common) {

    var page = window.location.pathname;
    page = page.substring(page.lastIndexOf("/") + 1, page.indexOf(".php")).toLowerCase();
    //alert(page);
    //var require_page = pageMap[page];

    if (page == 'm_index') {
        require(['js/m_controller']);
    }
    else {
        require(['js/controller']);
    }
});