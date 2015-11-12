<?php
/**
 * Created by PhpStorm.
 * User: vrushali
 * Date: 28/10/15
 * Time: 11:07 AM
 */
require_once 'lib/Mobile-Detect/Mobile_Detect.php';
$detect = new Mobile_Detect;


if ($detect->isMobile() || $detect->isTablet()) {
    header('Location: m_index.php');
    exit;
}
?>
<!DOCTYPE html>
<html>
<head>
    <title>Feeds</title>
    <link rel="stylesheet" type="text/css" href="lib/bootstrap/dist/css/bootstrap.min.css">
    <link rel="stylesheet" type="text/css" href="lib/jquery-ui/themes/base/jquery-ui.min.css">
    <link rel="stylesheet" type="text/css" href="css/feeds.css">
</head>
<body>

<ul id="container"></ul>

<script data-main="js/pageload" src="lib/require.js-2.1.16/require.js"></script>
</body>
</html>

