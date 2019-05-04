<?php
	$name = $_POST["name"];
$json_source = file_get_contents("jsonee/".$name.".json");
// Décode le JSON
$json_data = json_decode($json_source);
 ?>
 <div class="content">
	 <p><font size="6"><b><?php echo $json_data->name; ?></b> </font></p>
	<span class="image left">
  	<img src=<?php echo "images/".$name.".jpg"; ?> height="500" width="600">
	</span>
	<span class="image right">
</div>
<font class="align-center" size="5" face="Brush Script MT"><p><?php echo $json_data->description; ?></p></font>
<form>
