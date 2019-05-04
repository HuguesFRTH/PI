
<?php
$first = "neptune";
$json_source = file_get_contents("json/".$first.".json");
// Décode le JSON
$json_data = json_decode($json_source);
 ?>


<!DOCTYPE HTML>

<html>
	<head>
		<title>Site interdisciplinaire</title>
		<meta charset="utf-8" />
		<meta name="viewport" content="width=device-width, initial-scale=1" />
		<link rel="stylesheet" href="assets/css/main.css" />
	</head>
	<body class="subpage">

			<header id="header">
				<div class="logo"><a href="index.html">Site <span>by Thomas</span></a></div>
				<a href="#menu">Menu</a>
			</header>

			<nav id="menu">
				<ul class="links">
					<li><a href="index.html">Home</a></li>
					<li><a href="generic.php">Generic</a></li>

				</ul>
			</nav>

			<section id="One" class="wrapper style3">
				<div class="inner">
					<header class="align-center">
						<p>Le site interdisciplinaire</p>
						<h2>La présentation des différents éléments astronomiques</h2>
					</header>
				</div>
			</section>

      <CENTER><TABLE width=60% border=1>
<TR>
<TD width=33%>  <div align="right">  <label >Latitude<input type="text" name="latdeg" value="" size="2" style="width:15%"  minlength="4" maxlength="15"></label>
  <button><a class="favorite styled" href="latlong.htm" target="_blank"> <style type="text/css"> .text{ color: #3D59FF } </style> <span class="text">Plus</span> </a> </button> </div></TD>
<TD width=33%>  <div align="left">    <label>Longitude<input type="text" name="latdeg" value="" size="2" style="width:15%"  minlength="4" maxlength="15" ></label>
<button><a class="favorite styled" href="latlong.htm" target="_blank"><style type="text/css"> .text{ color: #3D59FF } </style> <span class="text">D'aide</span></a></button></div></TD>


</TR>

</TABLE>



</CENTER>

			<section id="two" class="wrapper style2">
				<div class="inner">
					<div class="box">
						<div class="content">
							<header class="align-center">
								<p><b>Les Planètes</b></p>
								<h2> </h2>
							</header>



              <?php
        				include("allPlanetes.php");
        			?>


							<header class="align-center">
								<p> <b>Les constellations </b></p>
								<h2> </h2>
							</header>

              <?php
                include("allconstellation.php");
              ?>

							<header class="align-center">
								<p><b>Les satellites </b></p>
								<h2> </h2>
							</header>

              <?php
                include("allsatellite.php");
              ?>



						</div>
					</div>
				</div>
			</section>
			<footer id="footer">
				<div class="container">
					<ul class="icons">
						<li><a href="#" class="icon fa-twitter"><span class="label">Twitter</span></a></li>
						<li><a href="#" class="icon fa-facebook"><span class="label">Facebook</span></a></li>
						<li><a href="#" class="icon fa-instagram"><span class="label">Instagram</span></a></li>
						<li><a href="#" class="icon fa-envelope-o"><span class="label">Email</span></a></li>
					</ul>
				</div>
				<div class="copyright">
					&copy; Pour nous contacter, cliquer sur les icones juste au dessus.
				</div>
			</footer>

			<script src="assets/js/jquery.min.js"></script>
			<script src="assets/js/jquery.scrollex.min.js"></script>
			<script src="assets/js/skel.min.js"></script>
			<script src="assets/js/util.js"></script>
			<script src="assets/js/main.js"></script>
