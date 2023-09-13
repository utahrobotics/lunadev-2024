from pypozyx import *

# Import Pose, Point and Quarternion as msg type, to differentiate from Pozyx classes of same name
import geometry_msgs.msg.Pose as MsgPose
import geometry_msgs.msg.Point as MsgPoint
import geometry_msgs.msg.Quarternion as MsgQuaternion

import yaml

class PozyxLocalizer:
    """
    Pozyx localizer class

    This class connects to a pozyx tag through a USB serial connection and returns the current position from the pozyx
    environment. 

    Attributes
    ----------
    deviceID: int
        The ID of the pozyx tag in hexadecimal or decimal form.
    pose: MsgPose
        The position and orientation of the pozyx device
    remoteID: int, optional
        The pozyx remote id, used to connect to multiple pozyx tags.
    remote: boolean, optional
        Set to True if using multiple tags. Defaults to False.
    algorithm: int, optional
        The type of pozyx positionging algorithm. POZYX_POS_ALG_UWB_ONLY is enabled by default. 
    anchors: DeviceCoordinates[]
        List of Coordinates objects. Coordinates is explaining the coordinates of the anchors in the Pozyx system. 
    dimension: int, optional
        The amount of dimentions to use. Either 2D, 3D, 5D. 
    height: int, optional
        The height of the Pozyx system in mm. This is only used when the dimentions is in 3D. Defaults to 1000mm.
    
    Methods
    -------
    parseYamlConfig(path)
        Parsing the configuration file to the self.anchors as a list of anchors.
    createSerialConnectionToTag(tagName=None)
        Creating a serial connection either automatically or manually with providing the tagName. Returns a PozyxSerial object.
    setAnchorsManually(saveToFlash=False)
        Using anchors that is provided in self.anchors.
    loop()
        Getting the position of the connected tag and returning it.
    posAndOrientatonToString():
        Returning a string with the x,y,z coordinates of the position and the orientation.


    """
    def __init__(self, anchors, port=None, remoteID=0x00, remote=False, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, height=1000):
        """
        Attributes
        ----------
        deviceID: int
            The ID of the pozyx tag in hexadecimal or decimal form.
        pose: MsgPose
            The position and orientation of the pozyx device
        remoteID: int, optional
            The pozyx remote id, used to connect to multiple pozyx tags.
        remote: boolean, optional
            Set to True if using multiple tags. Defaults to False.
        algorithm: int, optional
            The type of pozyx positionging algorithm. POZYX_POS_ALG_UWB_ONLY is enabled by default. 
        anchors: Coordinates[]
            List of Coordinates objects. Coordinates is explaining the coordinates of the anchors in the Pozyx system. 
        dimension: int, optional
            The amount of dimentions to use. Either 2D, 3D, 5D. 
        height: int, optional
            The height of the Pozyx system in mm. This is only used when the dimentions is in 3D. Defaults to 1000mm.
        """

        self.deviceID = None    # ID of the master device
        self.pose = MsgPose()   # Position and orientation of the Pozyx tag in a ROS Pose type

        self.pozyx = None           # Pozyx class
        self.remoteID = remoteID
        self.algorithm = algorithm
        self.dimension = dimension
        self.anchors = anchors
        self.height = height

        if type(self.anchors) == str: 
            self.parseYamlConfig(self.anchors)

        if not remote:
            self.remoteID = None

        if port is None:
            self.pozyx = self.createSerialConnectionToTag()
        else:
            self.pozyx = self.createSerialConnectionToTag(tagName=port)

        self.setAnchorsManually()

    def parseYamlConfig(self, path):
        """
        Parsing the configuration file to the self.anchors as a list of anchors. 

        Parameters
        ----------
        path: str
            The path of the config file.
        """
        anchors = []

        with open(path, "r") as file: 
            configYaml = yaml.safe_load(file)
            for anchor in configYaml["anchors"]:
                coordinates = Coordinates(anchor["coordinates"]["x"], anchor["coordinates"]["y"], anchor["coordinates"]["z"])
                dc = DeviceCoordinates(anchor["id"], anchor["flag"], coordinates)
                anchors.append(dc)

        self.anchors = anchors

    def createSerialConnectionToTag(self, tagName=None):
        """
        Creating a serial connection either automatically or manually with providing the tagName. Returns a PozyxSerial object.

        Paramters
        ---------
        tagName: 
            The name of the serial port. Use this variable for connecting to a USB port manually. If not used, this method will detect the USB port automatically. 
        """
        if tagName is None:
            serialPort = get_first_pozyx_serial_port()
            print('Auto assigning serial port: ' + str(serialPort))
        else:
            serialPort = tagName
        
        connection = PozyxSerial(serialPort)

        if connection is None:
            print('No Pozyx connected. Check if one is connected')

        return connection

    def setAnchorsManually(self, saveToFlash=False):
        """
        Using anchors that is provided in self.anchors. 

        Paramters
        ---------
        saveToFlash: boolean, optional
            If set to True. The tag will save the anchors posisions to it's flash memory. 
        """
        status = self.pozyx.clearDevices(self.remoteID)

        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remoteID)
        
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO,len(self.anchors), remote_id=self.remoteID)

        if saveToFlash:
            self.pozyx.saveAnchorIds(remote_id=self.remoteID)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remoteID)

        return status
        
    def loop(self):
        """
        Getting the position of the connected tag and returning it. 
        """
        # Define variable to store Position and orientation
        position = Coordinate()
        orientation = Quaternion()

        # Set position and orientation
        status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=self.remoteID)
        self.pozyx.getQuaternion(orientation, remote_id=self.remoteID)

        # Set ROS pose to values form Pozyx
        self.pose = Pose(
            MsgPoint(position.x, position.y, position.z),
            MsgQuaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        )

        if status == POZYX_SUCCESS: 
            print(self.posAndOrientatonToString())
        else: 
            statusString = "failure" if status == POZYX_FAILURE else "timeout"
            print('Error: Do positioning failed due to ' + statusString)

    def posAndOrientatonToString(self):
        """
        Returning a string with the x,y,z coordinates of the position and the orientation.
        """
        return 'Current position:\n  ' + str(self.position) + '\nCurrent orientation\n  ' + str(self.orientation) + '\n'

if __name__ == '__main__':
    anchors = [
        DeviceCoordinates(0x682c, 1, Coordinates(-1993, 1084, 0)),
        DeviceCoordinates(0x6869, 1, Coordinates(1954, -1739, 0)),
        DeviceCoordinates(0x680b, 1, Coordinates(-355, 1742, 0)),
        DeviceCoordinates(0x6851, 1, Coordinates(176, -3328, 0))
    ]

    localizer = PozyxLocalizer(anchors = "PozyxConfig.yaml")

    while True:
        localizer.loop()