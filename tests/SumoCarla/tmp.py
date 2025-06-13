import xml.etree.ElementTree as ET

rou_file = 'test_scenario_town04/Town04.rou.xml'
# find the vehicle
xmlTree = ET.parse(rou_file)
xmlRoot = xmlTree.getroot()
# Cycle through all vehicles defined in file.
for vehicle in xmlRoot.findall("vehicle"):
    # Looks for the car ID.
    vehicle.set('type', 'car')
xmlTree.write('Town04.rou.xml')