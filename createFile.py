import os
# creating an empty file, if its there we delete it
if os.path.isfile("./myFile.txt"):
    os.remove("./myFile.txt")
f = open("myFile.txt", "x")