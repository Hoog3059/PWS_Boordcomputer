import tkinter
import tkinter.filedialog

root = tkinter.Tk()
root.withdraw()

filename = tkinter.filedialog.askopenfilename()

with open(filename, mode="r+") as logfile:
    for i, line in enumerate(logfile.readlines()):
        if line.startswith("#"):
            continue

        data = line.split(";")

        if i == 0:
            print(";".join(data), end="")        
        elif i == 1:
            data[0] = "s"
            data[1] = "m/s^2"
            data[2] = "m/s^2"
            data[3] = "m/s^2"
            data[7] = "m/s"
            print(";".join(data), end="")
        else:
            data[0] = str(int(data[0]) / 1000)
            data[1] = str(int(data[1]) / 100)
            data[2] = str(int(data[2]) / 100)
            data[3] = str(int(data[3]) / 100)
            data[7] = str(float(data[7]) / 100)
            print(";".join(data), end="")
