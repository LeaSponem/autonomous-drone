"""
This code is to plot the logs produced by the code "test_stop_auto" in IRL Mode
"""

def read_and_plot(name):

    "------ List of the data ------"
    list_V_command = []
    list_V_measured = []
    list_measured_distance = []
    list_yaw = []
    list_time = []

    "------ Reading of the doc ------"

    doc = open(name, 'r')
    data = doc.readline()       #The first line should be "Time"
    assert data == "Time"

    while data =! "V_command":
        list_time.append(float(data))
        data = doc.readline()

    while data =! "V_measured":
        list_V_measured.append(float(data))
        data = doc.readline()

    while data =! "measured_distance":
        list_measured_distance.append(float(data))
        data = doc.readline()

    while data=!"":
        list_yaw.append(float(data))
        data = doc.readline()

    doc.close()

    "------ Plotting the lists ------"

    fig, axes = plt.subplots(nrows=1, ncols=2)
    title = "K=" + str(K)

    axes[0].plot(list_time,list_V_command, label="Ordered")
    axes[0].plot(list_time,list_V_measured, label="Measured")
    axes[0].set_xlabel('Time')
    axes[0].set_ylabel("Speed on x")
    axes[0].legend()

    axes[1].plot(list_time,list_measured_distance)
    axes[1].plot([list_time[0],list_time[-1]],[target_distance,target_distance])
    axes[1].set_xlabel("Time")
    axes[1].set_ylabel("Measured Distance")

    plt.title(title)
    plt.show()

