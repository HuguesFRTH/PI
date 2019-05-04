<?php
	$path = $_GET["name"];
$json_source = file_get_contents($path."/file.json");
// Décode le JSON
$json_data = json_decode($json_source);
 ?>

 <!DOCTYPE html>
<html>
		<head>
			<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
			<title><?php echo $json_data->nom; ?></title>
			<?php include "head.php" ?>
		</head>

		<body>
			<a>
				<h1 class="blocktext">
					<font size="7pt" face="Courier New, Courier, monospace">
						<?php echo $json_data->nom; ?>
					</font>
				</h1>
				<h1 class="blocktext">

					<font size="3pt" face="Courier New, Courier, monospace"></font>
				</h1>

			</a>
			<p>
				<span class="image left">
					<a href=<?php echo $path."/principale";?> target="_blank">
						<img class="imagec" src=<?php echo $path."/principale";?> />
					</a>
				</span>
				<span class="textedesc">
					<font size="5pt" face="Arial, Helvetica, sans-serif" color="#ECFFDD">
						<?php echo $json_data->description; ?>
					</font>
				</span>
			</p>


		</table>
	</table>


	</body>
</html>
