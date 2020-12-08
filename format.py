import tkinter
import tkinter.filedialog

root = tkinter.Tk()
root.withdraw()

filename = tkinter.filedialog.askopenfilename()

with open(filename, mode="r+") as logfile:
    for i, line in enumerate(logfile.readlines()):
        data = line.split(";")

        if i == 1:
            data[0] = "s"
            print(";".join(data), end="")
        elif i > 1:
            data[0] = str(int(data[0])/1000)
            print(";".join(data), end="")
        else:
            print(";".join(data), end="")
