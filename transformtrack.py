import xml.etree.ElementTree as ET


def getElements(tree):
    root = tree.getroot()
    elems = list()
    elems.append(root.find('./world[@name="default"]/model[1]/link[@name="link_ground"]/collision[@name="collision"]/geometry/plane/size'))
    elems.append(root.find('./world[@name="default"]/model[1]/link[@name="link_ground"]/visual[@name="visual"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[2]/link[@name="link_park"]/collision[@name="park"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[2]/link[@name="link_park"]/visual[@name="visual_park"]/geometry/mesh/scale'))
    for i in range(1,17):
        elems.append(root.find('./world[@name="default"]/model['+str(i+2)+']/link[@name="bumper_sensor"]/collision[@name="bumper_sensor"]/geometry/mesh/scale'))
        elems.append(root.find('./world[@name="default"]/model['+str(i+2)+']/link[@name="bumper_sensor"]/visual[@name="visual"]/geometry/mesh/scale'))
    return elems


def printElements(tree):
    elems = getElements(tree)
    trackCollision,trackVisual = elems[:2]
    sensors = elems[2:]
    print('Track Floor Collision: ' + trackCollision.text)
    print('Track Floor Visual Scaling: ' + trackVisual.text)
    it = iter(sensors)
    counter = 1
    for waypoint in it:
        print('Sensor '+str(counter)+' Collision Scaling: ' + waypoint.text)
        print('Sensor '+str(counter)+' Visual Scaling: ' + next(it).text)
        counter = counter + 1

def scaleElems(tree,x,y):
    elems = getElements(tree)
    trackCollision,trackVisual = elems[:2]
    sensors = elems[2:]
    #trackCollisionCoords = trackCollision.text.split(' ')
    #trackCollision.text = str(float(trackCollisionCoords[0])*x) + ' ' + str(float(trackCollisionCoords[1])*y)
    trackCollision.text = "9999 9999"
    trackVisual.text = str(x) + ' ' + str(y) + ' 1'
    it = iter(sensors)
    for sensor in it:
        try:    
                sensor.text = str(x) + ' ' + str(y) + ' 1'
        except:
                None
        try:
                next(it).text = str(x) + ' ' + str(y) + ' 1'
        except:
                None
    


tree = ET.parse('src/conde_simulator/conde_world/worlds/conde_world.world')
scaleElems(tree,2,5)
tree.write("transformed.world")