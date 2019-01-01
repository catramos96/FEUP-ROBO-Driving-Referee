import xml.etree.ElementTree as ET


def getElements(tree):
    root = tree.getroot()
    elems = list()
    elems.append(root.find('./world[@name="default"]/model[1]/link[@name="link_ground"]/collision[@name="collision"]/geometry/plane/size'))
    elems.append(root.find('./world[@name="default"]/model[1]/link[@name="link_ground"]/visual[@name="visual"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[2]/link[@name="link_park"]/collision[@name="park"]/geometry/plane/size'))
    elems.append(root.find('./world[@name="default"]/model[2]/link[@name="link_park"]/visual[@name="visual_park"]/geometry/plane/size'))
    elems.append(root.find('./world[@name="default"]/model[3]/link[@name="bumper_sensor"]/collision[@name="bumper_sensor"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[3]/link[@name="bumper_sensor"]/visual[@name="visual"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[4]/link[@name="bumper_sensor"]/collision[@name="bumper_sensor"]/geometry/mesh/scale'))
    elems.append(root.find('./world[@name="default"]/model[4]/link[@name="bumper_sensor"]/visual[@name="visual"]/geometry/mesh/scale'))
    for i in range(1,9):
        elems.append(root.find('./world[@name="default"]/model['+str(i+4)+']/link[@name="bumper_sensor"]/collision[@name="bumper_sensor"]/geometry/mesh/scale'))
        elems.append(root.find('./world[@name="default"]/model['+str(i+4)+']/link[@name="bumper_sensor"]/visual[@name="visual"]/geometry/mesh/scale'))
    return elems


def printElements(tree):
    elems = getElements(tree)
    trackCollision,trackVisual,parkCollision,parkVisual,boundsCollision,boundsVisual,semaphoreCollision,semaphoreVisual = elems[:8]
    waypoints = elems[8:]
    print('Track Floor Collision: ' + trackCollision.text)
    print('Track Floor Visual Scaling: ' + trackVisual.text)
    print('Park Floor Collision: ' + parkCollision.text)
    print('Park Floor Visual: ' + parkVisual.text)
    print('Track Bounds Collision Scaling: ' + boundsCollision.text)
    print('Track Bounds Visual: ' + boundsVisual.text)
    print('Semaphore Collision Scaling: ' + semaphoreCollision.text)
    print('Semaphore Visual Scaling: ' + semaphoreVisual.text)
    it = iter(waypoints)
    counter = 1
    for waypoint in it:
        print('Waypoint '+str(counter)+' Collision Scaling: ' + waypoint.text)
        print('Waypoint '+str(counter)+' Visual Scaling: ' + next(it).text)
        counter = counter + 1

def scaleElems(tree,x,y):
    elems = getElements(tree)
    trackCollision,trackVisual,parkCollision,parkVisual,boundsCollision,boundsVisual,semaphoreCollision,semaphoreVisual = elems[:8]
    waypoints = elems[8:]


tree = ET.parse('src/conde_simulator/conde_world/worlds/conde_world.world')
scaleElems(tree,1,1)
tree.write("transformed.world")