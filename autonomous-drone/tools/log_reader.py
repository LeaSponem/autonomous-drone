"""
This code is to plot the logs produced by the code "test_stop_auto" in IRL Mode
"""

import matplotlib.pyplot as plt

def read_and_plot(name, K):
    
    target_distance = 200      #The drone must stop at this distance from the obstacle

    "------ List of the data ------"
    list_V_command = []
    list_V_measured = []
    list_measured_distance = []
    list_yaw = []
    list_time = []

    "------ Reading of the doc ------"

    doc = open(name, 'r')
    data = doc.readline()       #The first line should be "Time"
    
    assert data == "Time \n"
    
    data = doc.readline()

    while data != "V_command \n":
        list_time.append(float(data[:-1]))
        data = doc.readline()
        
    data = doc.readline()

    while data != "V_measured \n":
        list_V_command.append(float(data[:-1]))
        data = doc.readline()
    
    data = doc.readline()

    while data != "measured_distance \n":
        list_V_measured.append(float(data[:-1]))
        data = doc.readline()
    
    data = doc.readline()

    while data != "yaw \n":
        list_measured_distance.append(float(data[:-1]))
        data = doc.readline()
    
    data = doc.readline()
    
    while data != "":
        list_yaw.append(float(data[:-1]))
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

    
    """ Rescale the graph
    stop = int(len(list_time)/3)
    axes[1].plot(list_time[1:stop],list_measured_distance[1:stop])
    axes[1].plot([list_time[0],list_time[stop]],[target_distance,target_distance])
    axes[1].set_xlabel("Time")
    axes[1].set_ylabel("Measured Distance")
    """
    plt.title(title)
    plt.show()

