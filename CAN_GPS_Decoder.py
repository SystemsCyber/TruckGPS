import os
import matplotlib.pyplot as plt
import plotly.graph_objs as go
import plotly.subplots as sp
import argparse
import struct
from collections import Counter
from numpy import mean
import numpy as np

#Gets highest occuring value in a list/array
def most_frequent_value(lst):
    counter = Counter(lst)
    most_common = counter.most_common(1)
    
    if most_common:
        return most_common[0][0]
    else:
        return None  # List is empty

#Converts the CAN Binary format into a candump format
def convert_bin_to_candump(filename):
    candump_list =[]
    fileLocation = 0
    file_size = os.path.getsize(filename)
    with open(filename,'rb') as binFile:
        while(fileLocation<file_size):
            block =binFile.read(512) #read every 512 bytes
            fileLocation+=512
            for recordNum in range(19): #Parse through CAN message
                record = block[4+recordNum*25:4+(recordNum+1)*25]
                channel = record[0]
                timeSeconds = struct.unpack("<L",record[1:5])[0]
                timeMicrosecondsAndDLC = struct.unpack("<L",record[13:17])[0]
                timeMicroseconds = timeMicrosecondsAndDLC & 0x00FFFFFF
                abs_time = timeSeconds + timeMicroseconds * 0.000001
                ID = struct.unpack("<L",record[9:13])[0]
                message_bytes = record[17:25]
                #create list for all data parsed
                candump_list.append("({:0.6f}) can{:0.0f} {:08X}#{}"
                                    .format(abs_time,channel,ID,''.join(["{:02X}"
                                        .format(b) for b in message_bytes])))
    return candump_list

def twos_complement(hex_string, num_bits):
    # Convert hex to binary
    binary_string = bin(int(hex_string, 16))[2:].zfill(num_bits)
    
    # Check if it's a negative number
    if binary_string[0] == '1':
        inverted_bits = ''.join('1' if bit == '0' else '0' for bit in binary_string)
        incremented_value = int(inverted_bits, 2) + 1
        decimal_value = -incremented_value
    else:
        decimal_value = int(binary_string, 2)
    return decimal_value

#Generates a kml file to be used for Google Earth
def create_kml_file(filename, longitude_list, latitude_list):
    with open(filename, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('  <Document>\n')
        
        # Define a style for a smaller pin
        f.write('    <Style id="smallPin">\n')
        f.write('      <IconStyle>\n')
        f.write('        <scale>0.5</scale>\n')  # This scale value can be adjusted to your desired size
        f.write('        <Icon>\n')
        f.write('          <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n')  # default yellow pushpin
        f.write('        </Icon>\n')
        f.write('      </IconStyle>\n')
        f.write('    </Style>\n')

        for i, (longitude, latitude) in enumerate(zip(longitude_list, latitude_list), start=1):
            f.write(f'    <Placemark>\n')
            f.write(f'      <name>Location {i}</name>\n')
            f.write(f'      <styleUrl>#smallPin</styleUrl>\n')  # Reference the custom style
            f.write(f'      <Point>\n')
            f.write(f'        <coordinates>{longitude},{latitude},0</coordinates>\n')
            f.write(f'      </Point>\n')
            f.write(f'    </Placemark>\n')

        f.write('  </Document>\n')
        f.write('</kml>\n')


#Decodes CAN data
def get_data(can_msgs):
    timestampsVehicle = []
    timestampsEngine = []
    vehicleSpeed = []
    engineSpeed = []
    longitude = []
    latitude = []
    gpsSpeed = []
    headingDegree = []
    timestampHeading = []
    numSats = []
    gpsTime = []
    timestampSat = []
    timestampsLocation = []
    counter = 0
    firstTimestamp = 0
    canloggerTime = []
    
    for i in can_msgs:
        contents = i.split()
        if counter == 0:
            firstTimestamp = float(contents[0][1:-1])
            counter = 1
        data = contents[2].split('#')
        id = data[0]
        dataBytes = data[1]

        if(len(id) < 8):
            continue
        if(len(dataBytes) < 16):
            continue

        #Get Wheel Based Vehicle Speed 84
        if(id[2:-2] == "FEF1"):
            speed = int((dataBytes[4:6] + dataBytes[2:4]),16)*(1/256)
            if(0 < speed < 250.99609375):
                vehicleSpeed.append(speed)
                timestampsVehicle.append(float(contents[0][1:-1]) - firstTimestamp)

        #Get Engine Speed 190
        if(id[2:-2] == "F004"):
            speed = int((dataBytes[8:10] + dataBytes[6:8]),16)*0.125
            if(0 < speed < 8031.875):
                engineSpeed.append(speed)
                timestampsEngine.append(float(contents[0][1:-1]) - firstTimestamp)

        #Get Longitude and Latitude
        if(id[2:-2] == "FF01"):
            longitudeHex = dataBytes[14:16] + dataBytes[12:14] + dataBytes[10:12] + dataBytes[8:10]
            latitudeHex = dataBytes[6:8] + dataBytes[4:6] + dataBytes[2:4] + dataBytes[0:2]
            longitude.append(twos_complement(longitudeHex, 32)/10000000)
            latitude.append(twos_complement(latitudeHex, 32)/10000000)
            timestampsLocation.append(float(contents[0][1:-1]) - firstTimestamp)

        #Get GPS Speed and Heading Degree
        if(id[2:-2] == "FF02"):
            gpsSpeed.append(int((dataBytes[6:8] + dataBytes[4:6] + dataBytes[2:4] + dataBytes[0:2]), 16)/1000000)
            headingDegree.append(int((dataBytes[14:16] + dataBytes[12:14] + dataBytes[10:12] + dataBytes[8:10]), 16)/100000)
            timestampHeading.append(float(contents[0][1:-1]) - firstTimestamp)

        #Get number of satellites and GPS time
        if(id[2:-2] == "FF03"):
            numSats.append(int(dataBytes[0:2], 16))
            us = int((dataBytes[6:8] + dataBytes[4:6] + dataBytes[2:4]), 16)
            epoch = int((dataBytes[14:16] + dataBytes[12:14] + dataBytes[10:12] + dataBytes[8:10]), 16)
            gpsTime.append(float(epoch) + (float(us)/1000000))
            canloggerTime.append(float(contents[0][1:-1]))
            timestampSat.append(float(contents[0][1:-1]) - firstTimestamp)
                
    return [timestampsVehicle, vehicleSpeed, timestampsEngine, engineSpeed,
        longitude, latitude, gpsSpeed, headingDegree, numSats, gpsTime, timestampSat, 
        timestampHeading, timestampsLocation, canloggerTime]

def handleFile(filename):
    #Handle .txt file
    if(filename[-4:] == ".txt"):
        with open(filename) as f:
            can_msgs = f.readlines()
        canData = get_data(can_msgs)
        return canData

    #Handle .log file
    elif(filename[-4:] == ".log"):
        with open(filename) as f:
            can_msgs = f.readlines()
        canData = get_data(can_msgs)
        return canData

    #Handle .bin file
    elif(filename[-4:] == ".bin"):
       canData = get_data(convert_bin_to_candump(filename))
       return canData

    #Exception
    else:
        print("This decoder only handles txt, log, and bin files.")
        exit()


def main():
    #Argument options
    parser = argparse.ArgumentParser(description='Generate KML file, plot coordinates, and show GPS info')
    parser.add_argument('--plot', nargs=1, metavar='file', help='Input a CAN binary or CAN text file to plot vehicle and engine speed')
    parser.add_argument('--show-gps-info', nargs=1, metavar='file', help='Input a CAN binary or CAN text file to show GPS info')
    parser.add_argument('--create-kml-file', nargs=1, metavar='file', help='Creates a klm based on the longitude and latitude values in the CAN logs.')
    parser.add_argument('--plot-two-files', nargs=2, metavar=('file1', 'file2'), help='Plot engine speed and vehicle speed from two CAN files')

    args = parser.parse_args()
    
    if args.plot:
        # Handle file
        filename = args.plot[0]
        canData = handleFile(filename)

        timestampsVehicle = canData[0]
        vehicleSpeed = canData[1]
        timestampsEngine = canData[2]
        engineSpeed = canData[3]

        longitude = canData[4]
        latitude = canData[5]
        timestampsLocation = canData[12]

        gpsSpeed = canData[6]
        headingDegree = canData[7]
        timestampHeading = canData[11]

        numSats = canData[8]
        gpsTime = canData[9]
        canloggerTime = canData[13]
        timestampSats = canData[10]


        # Create subplots
        fig = sp.make_subplots(rows=4, cols=2, subplot_titles=('Wheel Based Vehicle Speed SPN 84', 'Engine Speed SPN 190',
                                                            'Longitude over Time', 'Latitude over Time',
                                                            'GPS Speed', 'Heading Degree', 'Number of Satellites',
                                                          'GPS Tracked Time over CANLogger3/TruckCape Tracked Time'))

        # Plot for Vehicle Speed
        vehicle_speed_trace = go.Scatter(x=timestampsVehicle, y=vehicleSpeed, mode='lines', name='Vehicle Speed')
        fig.add_trace(vehicle_speed_trace, row=1, col=1)
        fig.update_xaxes(title_text='Time', row=1, col=1)
        fig.update_yaxes(title_text='Vehicle Speed (km/h)', row=1, col=1)

        # Plot for Engine Speed
        engine_speed_trace = go.Scatter(x=timestampsEngine, y=engineSpeed, mode='lines', name='Engine Speed')
        fig.add_trace(engine_speed_trace, row=1, col=2)
        fig.update_xaxes(title_text='Time', row=1, col=2)
        fig.update_yaxes(title_text='Engine Speed (rpm)', row=1, col=2)

        # Plot for Longitude over Time
        longitude_trace = go.Scatter(x=timestampsLocation, y=longitude, mode='lines', name='Longitude')
        fig.add_trace(longitude_trace, row=2, col=1)
        fig.update_xaxes(title_text='Time', row=2, col=1)
        fig.update_yaxes(title_text='Longitude', row=2, col=1)

        # Plot for Latitude over Time
        latitude_trace = go.Scatter(x=timestampsLocation, y=latitude, mode='lines', name='Latitude')
        fig.add_trace(latitude_trace, row=2, col=2)
        fig.update_xaxes(title_text='Time', row=2, col=2)
        fig.update_yaxes(title_text='Latitude', row=2, col=2)

        # Plot for GPS Speed
        gps_speed_trace = go.Scatter(x=timestampHeading, y=gpsSpeed, mode='lines', name='GPS Speed')
        fig.add_trace(gps_speed_trace, row=3, col=1)
        fig.update_xaxes(title_text='Time', row=3, col=1)
        fig.update_yaxes(title_text='GPS Speed (m/h)', row=3, col=1)

        # Plot for Heading Degree
        heading_degree_trace = go.Scatter(x=timestampHeading, y=headingDegree, mode='lines', name='Heading Degree')
        fig.add_trace(heading_degree_trace, row=3, col=2)
        fig.update_xaxes(title_text='Time', row=3, col=2)
        fig.update_yaxes(title_text='Heading Degree', row=3, col=2)

        # Plot for Number of Satellites
        num_sats_trace = go.Scatter(x=timestampSats, y=numSats, mode='lines', name='Number of Satellites')
        fig.add_trace(num_sats_trace, row=4, col=1)
        fig.update_xaxes(title_text='Time', row=4, col=1)
        fig.update_yaxes(title_text='Number of Satellites', row=4, col=1)

        # Plot for GPS Time over Timestamp Satellites
        gps_time_trace = go.Scatter(x=canloggerTime, y=gpsTime, mode='lines', name='GPS Time')
        fig.add_trace(gps_time_trace, row=4, col=2)
        fig.update_xaxes(title_text='Timestamp Satellites', row=4, col=2)
        fig.update_yaxes(title_text='GPS Time', row=4, col=2)

        # Update layout and display the plot
        fig.update_layout(title_text='GPS Plots', showlegend=True)
        fig.show()



    if args.show_gps_info:
        #Handle file
        filename = args.show_gps_info[0]
        canData = handleFile(filename)

        gpsSpeed = canData[6]
        headingDegree = canData[7]
        numSats = canData[8]
        gpsTime = canData[9]

        gpsRange = gpsTime[-1] - gpsTime[0]

        highestSats = max(numSats)
        lowestSats = min(numSats)
        mostFrequentSat = most_frequent_value(numSats)

        highestSpeed = max(gpsSpeed)
        lowestSpeed = min(gpsSpeed)
        meanSpeed = mean(gpsSpeed)

        highestHeading = max(headingDegree)
        lowestHeading = min(headingDegree)
        meanHeading = mean(headingDegree)

        print("When logging CAN data, the gps time ran for", gpsRange, "seconds starting at", gpsTime[0], "and ending at", gpsTime[-1])
        print("\nThe number of satellites ranged from", lowestSats, "to", highestSats, "with", mostFrequentSat, "being the most likely number of satellites present at a given time")
        print("\nThe gps velocity ranged from", lowestSpeed, "kph to", highestSpeed, "kph with the average velocity being", meanSpeed, "kph")
        print("\nThe heading degree averaged at", meanHeading, "degrees")
    
    if args.create_kml_file:
        #Handle file
        filename = args.create_kml_file[0]
        canData = handleFile(filename)

        longitude = canData[4]
        latitude = canData[5]

        kml_file_name = filename[:-4] + ".kml"
        create_kml_file(kml_file_name, longitude, latitude)
        print(f'KML file "{kml_file_name}" created successfully.')

    if args.plot_two_files:
        file1, file2 = args.plot_two_files

        canData1 = handleFile(file1)
        canData2 = handleFile(file2)

        #First file vehicle and engine speed
        timestampsVehicle1 = canData1[0]
        vehicleSpeed1 = canData1[1]
        timestampsEngine1 = canData1[2]
        engineSpeed1 = canData1[3]

        #Second file vehicle and engine speed
        timestampsVehicle2 = canData2[0]
        vehicleSpeed2 = canData2[1]
        timestampsEngine2 = canData2[2]
        engineSpeed2 = canData2[3]

        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(15, 7))

        # Plot for File 1 Vehicle Speed
        axes[0][0].plot(timestampsVehicle1, vehicleSpeed1, label='Wheel Based Vehicle Speed 84')
        axes[0][0].set_xlabel('Time')
        axes[0][0].set_ylabel('Vehicle Speed (km/h)')
        axes[0][0].set_title(file1 + ' Vehicle Speed Plot')
        axes[0][0].legend()
        axes[0][0].grid(True)

        # Plot for File 1 Engine Speed
        axes[0][1].plot(timestampsEngine1, engineSpeed1, label='Engine Speed 190')
        axes[0][1].set_xlabel('Time')
        axes[0][1].set_ylabel('Engine Speed (rpm)')
        axes[0][1].set_title(file1 + ' Engine Speed')
        axes[0][1].legend()
        axes[0][1].grid(True)

        # Plot for File 2 Vehicle Speed
        axes[1][0].plot(timestampsVehicle2, vehicleSpeed2, label='Vehicle Speed')
        axes[1][0].set_xlabel('Time')
        axes[1][0].set_ylabel('Vehicle Speed (km/h)')
        axes[1][0].set_title(file2 + ' Vehicle Speed Plot')
        axes[1][0].legend()
        axes[1][0].grid(True)

        # Plot for File 2 Engine Speed
        axes[1][1].plot(timestampsEngine2, engineSpeed2, label='Engine Speed')
        axes[1][1].set_xlabel('Time')
        axes[1][1].set_ylabel('Engine Speed (rpm)')
        axes[1][1].set_title(file2 + ' Engine Speed')
        axes[1][1].legend()
        axes[1][1].grid(True)

        # Display the plots
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()