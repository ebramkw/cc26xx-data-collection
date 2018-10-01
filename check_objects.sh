echo "";
while read line_from_file;
	do grep $line_from_file $1 | tail -1;
done < objects
echo "";
