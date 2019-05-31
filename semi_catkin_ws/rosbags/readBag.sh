newDir="${1%.*}"
filename="${1}"
echo "Creating new directory: ${newDir}"
echo "Exporting ${filename} to .csv files in new directory"
if [ -d "$newDir" ]
then
   rm -r "$newDir"
fi

mkdir $newDir

for topic in `rostopic list -b ${filename}`;
   do rostopic echo -p -b ${filename} $topic >${newDir}//${topic//\//_}.csv;
   done

