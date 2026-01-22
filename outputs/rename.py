import os


# this .py file should at the same repository as the folder ‘solutions’
def rename(folder="./solutions"):
    for filename in os.listdir(folder):
        src = f"{folder}/{filename}"
        filename = filename.replace(filename[12], ":")
        dst = f"{folder}/{filename}"
        os.rename(src, dst)


if __name__ == "__main__":
    rename()
