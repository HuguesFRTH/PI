<?php
	$name = $_POST["name"];
$json_source = file_get_contents("json/".$name.".json");
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
  <input name=<?php echo "\"".$json_data->name."\"";?> type="button" value=<?php echo "\"Observer ".$json_data->name."\"";?> >
</form>
<script>
var <?php echo $name; ?> = document.querySelector("input[name=<?php echo "'".$json_data->name."'"."]";?>");
<?php echo $name; ?>.addEventListener('click', <?php echo "function".$json_data->name; ?>);

function <?php echo "function".$json_data->name; ?>() {
if (<?php echo $name; ?>.value == <?php echo "\"Observer ".$json_data->name."\"";?>) {
	var xhr = new XMLHttpRequest();
	xhr.open('POST', "http://http://127.0.0.1:1042/[PACKET][COM]/longitude=0/latitude=45/astreID=2/");
}
}
}
</script>
