#!/bin/sh

mkdir linux_practice
cd linux_practice
mkdir name
cd name
touch file1.txt file2.txt
cd ..
mkdir permission
cd permission
touch file3.txt file4.txt
cd ..
cd name
rm file1.txt
mv file2.txt show.txt
echo "Hello linux" > show.txt
cat show.txt
cd ..
cd permission
mkdir subdir

for file in *; do
    # if [ -f "$file" ]; then
        chmod 644 "$file"
        echo "Changed permissions for $file to -rw-r--r--"
    # else
        # echo "$file is not a regular file."
    # fi
done
