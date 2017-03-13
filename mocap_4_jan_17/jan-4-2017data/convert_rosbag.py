'''
This function is based on bag2csv.py by Nick Speal
'''

import rosbag, sys, csv
import time
import string
import os

def bag2csv(bagfile):
    print "reading file " + bagfile
    # access bag
    bag = rosbag.Bag(bagfile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    # get list of topics from the bag
    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    # write a different CSV file for each topic
    for topicName in listOfTopics:
        filename = bagfile[0:-4] + string.replace(topicName,'/','_') + '.csv'
        with open(filename, 'w+') as csvfile:
            filewriter = csv.writer(csvfile, delimiter = ',')
            firstIteration = True	#allows header row
            for subtopic, msg, t in bag.read_messages(topicName):
                # for each instant in time that has data for topicName
                # parse data from this instant, which is of the form of
                # multiple lines of "Name: value\n"
                #	- put it in the form of a list of 2-element lists
                msgString = str(msg)
                msgList = string.split(msgString, '\n')
                instantaneousListOfData = []
                for nameValuePair in msgList:
                    splitPair = string.split(nameValuePair, ':')
                    for i in range(len(splitPair)):	#should be 0 to 1
                        splitPair[i] = string.strip(splitPair[i])
                    instantaneousListOfData.append(splitPair)
                # write the first row from the first element of each pair
                if firstIteration:	# header
                    headers = ["rosbagTimestamp"]	#first column header
                    for pair in instantaneousListOfData:
                        headers.append(pair[0])
                    filewriter.writerow(headers)
                    firstIteration = False
                # write the value from each pair to the file
                values = [str(t)]	# first column will have rosbag timestamp
                for pair in instantaneousListOfData:
                    values.append(pair[1])
                filewriter.writerow(values)
    bag.close()
