<?php
	$name = $_POST["name"];
$json_source = file_get_contents("json/".$name.".json");
// Décode le JSON
$json_data = json_decode($json_source);
 ?>
 <div class="content">
	 <p><font size="6"><b><?php echo $json_data->name; ?></b> </font></p>
	<span class="image right">
</div>
<font class="align-center" size="5" face="Brush Script MT"><p><?php echo $json_data->all; ?></p></font>
<form>
  <input name=<?php echo "\"".$json_data->name."\"";?> type="button" value=<?php echo "\"Observer ".$json_data->name."\"";?> >
</form>
