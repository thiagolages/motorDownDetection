#!/home/thiago/projects/venv3-speedbird/bin/python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

files = ["logs/ATT_00000042-0012.csv", "logs/ATT_00000172.csv"]
typesOfFlight = ["Normal", "Catastrophic"]

for file, typeOfFlight in zip(files, typesOfFlight):
    df = pd.read_csv(file)

    errorRoll   = df.DesRoll  - df.Roll
    errorPitch  = df.DesPitch - df.Pitch
    errorYaw    = df.DesYaw   - df.Yaw

    col_nums = np.array(range(1,len(errorRoll)+1, 1))

    errorRoll   = errorRoll.to_numpy()
    errorPitch  = errorPitch.to_numpy()
    errorYaw    = errorYaw.to_numpy()

    frame = {"nums": col_nums, "errorRoll" : errorRoll, "errorPitch" : errorPitch, "errorYaw" : errorYaw}
    error_df = pd.DataFrame(frame)

    # print(errorRoll.shape)
    # print(errorPitch.shape)
    # print(errorYaw.shape)

    fig, ax = plt.subplots(figsize=(12, 6))

    # error_df.plot.scatter(x="nums",  y="errorRoll"    ,  color="red"  , label="errorRoll" , marker=".", linewidths=0.5)
    # error_df.plot.scatter(x="nums",  y="errorPitch"   ,  color="green", label="errorPitch", marker=".", linewidths=0.5)
    # error_df.plot.scatter(x="nums",  y="errorYaw"     ,  color="blue" , label="errorYaw"  , marker=".", linewidths=0.5)

    ax.plot(errorRoll)
    ax.plot(errorPitch)
    ax.plot(errorYaw)


    plt.title(typeOfFlight + " Flight")
    plt.xlim([2490, 2510])
    plt.ylabel("Angle Error (degrees)")
    plt.legend(["Roll", "Pitch", "Yaw"])
    plt.grid("on")
    #plt.hold(False)

plt.show()