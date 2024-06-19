import csv 
import xml.etree.ElementTree as ET 
def main(xml_file,csv_file):

    # Parse the XML file
    tree = ET.parse(xml_file) 
    root = tree.getroot()
    
    # Find all Conf elements
    conf_elements = root.findall(".//Conf")

    # Open CSV file for writing
    with open(csv_file, "w", newline='') as csvfile:
        csvwriter = csv.writer(csvfile)

        # Write components to CSV file
        for conf in conf_elements:
            components = conf.text.strip().replace("[", "").replace("]", "").replace("'", "").split()
            components_str = ', '.join(components[:6])
            csvwriter.writerow(components_str.split(', ')[:6])

xml_file = 'taskfile_tampconfig_staircase_rob1.xml'
csv_file = '/home/alumne/catkin_wsTAMP/src/task_and_motion_planning/ktmpb/demos/OMPL_geo_demos/staircase/taskfile_tampconfig_staircase.csv'

main(xml_file,csv_file)
