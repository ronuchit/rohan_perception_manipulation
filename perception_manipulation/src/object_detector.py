#import tabletop_actions
import rospy

from object_manipulation_msgs.srv import FindClusterBoundingBox2, FindClusterBoundingBox2Request, FindClusterBoundingBox2Response
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest, FindClusterBoundingBoxResponse
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest, TabletopDetectionResponse
from tabletop_object_detector.srv import TabletopSegmentation, TabletopSegmentationResponse
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
from household_objects_database_msgs.msg import DatabaseModelPoseList
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.msg as sm

import random
from rospy.service import ServiceException
from visualization_msgs.msg import Marker
import math
import numpy as np
from tf import transformations

def xyzrgb2pc(xyz,bgr,frame_id):
    height= xyz.shape[0]
    width = xyz.shape[1]
    assert bgr.shape[0] == height
    assert bgr.shape[1] == width

    arr = np.empty((height,width,8),dtype='float32')
    arr[:,:,0:3] = xyz
    bgr1 = np.empty((height,width,4),dtype='uint8')
    bgr1[:,:,0:3] = bgr
    arr[:,:,4] = bgr1.view(dtype='float32').reshape(height, width)
    data = arr.tostring()
    msg = sm.PointCloud2()
    msg.data = data
    msg.header.frame_id = frame_id
    msg.fields = [sm.PointField(name='x',offset=0,datatype=7,count=1),
                  sm.PointField(name='y',offset=4,datatype=7,count=1),
                  sm.PointField(name='z',offset=8,datatype=7,count=1),
                  sm.PointField(name='rgb',offset=16,datatype=7,count=1)]
    msg.is_dense = False
    msg.width=width
    msg.height=height
    msg.header.stamp = rospy.Time.now()
    msg.point_step = 32
    msg.row_step = 32 * width
    msg.is_bigendian = False

    return msg

def PointCloud_to_PointCloud2(pc):
    isinstance(pc, PointCloud)
    xyz = np.array([[pt.x, pt.y, pt.z] for pt in pc.points])[None,:,:]
    rgb = np.zeros(xyz.shape)
    pc2 = xyzrgb2pc(xyz, rgb, pc.header.frame_id)
    return pc2

# class ObjectDetector(object):
#     def __init__(self):

#         narrow_detector = "object_detection"
#         rospy.loginfo("waiting for %s service" % narrow_detector)
#         rospy.wait_for_service(narrow_detector)
#         self.narrow_detector =  rospy.ServiceProxy(narrow_detector,
#                 TabletopDetection)

#         wide_detector = "wide_object_detection"
#         rospy.loginfo("waiting for %s service" % wide_detector)
#         rospy.wait_for_service(wide_detector)
#         self.wide_detector =  rospy.ServiceProxy(wide_detector,
#                 TabletopDetection)

#         box_detector = "find_cluster_bounding_box"
#         rospy.loginfo("waiting for %s service" % box_detector)
#         rospy.wait_for_service(box_detector)
#         self.box_detector =  rospy.ServiceProxy(box_detector,
#                 FindClusterBoundingBox)

#         collision_processing = "/tabletop_collision_map_processing/tabletop_collision_map_processing"
#         rospy.loginfo("Waiting for collision processing service to come up")
#         rospy.wait_for_service(collision_processing)
#         self.collision_processing = rospy.ServiceProxy(collision_processing,
#                 TabletopCollisionMapProcessing)

#         self.box_drawer = rospy.Publisher("box_drawer",
#                                           Marker
#                                           )

#         self.last_wide_msg = None
#         self.last_narrow_msg = None
#         self.last_detection_msg = None
#         self.last_box_msg = None
#         self.last_collision_processing_msg = None

#         rospy.loginfo("ObjectDetector is ready")

#     def __table_test(self, detection_msg):
#         table = detection_msg.detection.table
#         table_quat = (table.pose.pose.orientation.x,
#                       table.pose.pose.orientation.y,
#                       table.pose.pose.orientation.z,
#                       table.pose.pose.orientation.w)
#         table_angles = transformations.euler_from_quaternion(table_quat)
#         if (math.fabs(table_angles[0]) < 0.1 and
#             math.fabs(table_angles[1]) < 0.1):
#             return True
#         else:
#             rospy.logwarn("Table has wrong angles: %s",str(table_angles))
#             return False

#     def __detect(self, detector):
#         req = TabletopDetectionRequest()
#         req.return_clusters = True
#         req.return_models = True
#         try:
#             reply = detector(req)
#         except rospy.ServiceException, e:
#             rospy.logerr("error when calling object_detection: %s"%e)
#             return None
#         if reply.detection.result != reply.detection.SUCCESS:
#             return None
#         if len(reply.detection.clusters) == 0:
#             return None
#         if not self.__table_test(reply):
#             return None
#         return reply

#     def detect_narrow(self):
#         """Detects an object using the narrow stereo camera.
#         If an object is found it returns the resulting
#         tabletop_object_detector/TabletopDetectionResponse msg,
#         None otherwise
#         """
#         self.last_narrow_msg = self.__detect(self.narrow_detector)
#         self.last_detection_msg = self.last_narrow_msg
#         return self.last_narrow_msg

#     def detect_wide(self):
#         """Detects an object using the wide stereo camera.
#         If an object is found it returns the resulting
#         tabletop_object_detector/TabletopDetectionResponse msg,
#         None otherwise
#         """
#         self.last_wide_msg = self.__detect(self.wide_detector)
#         self.last_detection_msg = self.last_wide_msg
#         return self.last_wide_msg

#     def find_biggest_cluster(self, clusters):
#         '''Select the biggest cluster among clusters. It uses
#         detect_bounding_box to find the cluster position

#         Params:
#         clusters: a list of PointCloud among wich to find the closest
#         cluster.

#         A tipical usage is to call res = detect_[narrow|wide], and then select
#         the cluster using find_biggest_cluster(res.detection.clusters)
#         '''
#         if clusters is None:
#             return None

#         if len(clusters) == 0:
#             rospy.logerr("No object found!")
#             return

#         #finding the biggest cluster
#         #TODO use the builtin max function
#         max_len = 0
#         index = 0
#         for i,c in enumerate(clusters):
#             if len(c.points) > max_len:
#                 max_len = len(c.points)
#                 index = i
#         object_cluster = clusters[index]
#         rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points)))
#         return object_cluster

#     def find_random_cluster(self, clusters):
#         '''Selects a random cluster.

#         Parameters:
#         clusters: a list of PointCloud among wich to find the closest
#         cluster.

#         A tipical usage is to call res = detect_[narrow|wide], and then select
#         the cluster using find_random_cluster(res.detection.clusters)
#         '''
#         if clusters is None:
#             return None

#         if len(clusters) == 0:
#             rospy.logerr("No object found!")
#             return None

#         #using a random cluster
#         index = random.randint(0, len(clusters)-1)
#         object_cluster = clusters[index]
#         rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points)))
#         return object_cluster

#     def find_closest_cluster(self, clusters):
#         '''Searches for the closest cluster among clusters. It uses
#         detect_bounding_box to find the cluster position

#         Parameters:
#         clusters: a list of PointCloud among wich to find the closest
#         cluster.

#         A tipical usage is to call res = detect_[narrow|wide], and then select
#         the cluster using find_closest_cluster(res.detection.clusters)
#         '''
#         if clusters is None:
#             return None

#         if len(clusters) == 0:
#             rospy.logerr("No object found!")
#             return None

#         boxes = []
#         for cluster in clusters:
#             box_msg = self.detect_bounding_box(cluster)
#             if box_msg is not None:
#                 boxes.append(box_msg)

#         if len(boxes) == 0:
#             return None

#         closest_index = 0
#         closest_dist = 10000

#         for i, box in enumerate(boxes):
#             dist = math.sqrt(box.pose.pose.position.x**2 +
#                              box.pose.pose.position.y**2)
#             if dist < closest_dist:
#                 closest_dist = dist
#                 closest_index = i

#         rospy.loginfo("Using object %d with %f distance"%
#                       (closest_index, closest_dist))
#         return clusters[closest_index]


#     def call_collision_map_processing(self, detection_result):

#         if detection_result is None:
#             rospy.logerr("Error: using a None detection_result")
#             return None

#         rospy.loginfo("Calling collision map processing")
#         processing_call = TabletopCollisionMapProcessingRequest()
#         processing_call.detection_result = detection_result.detection
#     #    ask for the exising map and collision models to be reset
#         processing_call.reset_attached_models = True
#         processing_call.reset_collision_models = True
#         processing_call.desired_frame = "base_link"

#         try:
#             self.last_collision_processing_msg = self.collision_processing.call(processing_call)
#         except ServiceException, e:
#             rospy.logerr("Error calling collision map: %s" % str(e))
#             self.last_collision_processing_msg = None
#         return self.last_collision_processing_msg

#     def try_to_detect(self):
#         """Tries to detect an object using either the narrow or the wide
#         stereo. It returns the tabletop_object_detector/TabletopDetectionResponse
#         reply, or None if no object is found.
#         """

#         rospy.loginfo("Trying the narrow stereo...")
#         res_narrow = self.detect_narrow()
#         if res_narrow is None:
#             rospy.logwarn("No luck with narrow stereo, trying the wide one")
#             self.detect_wide()
#         if self.last_detection_msg is not None:
#             self.call_collision_map_processing(self.last_detection_msg)
#         return self.last_detection_msg


#     def detect_bounding_box(self, cluster = None,
#                             cluster_choser = "find_random_cluster"):
#         """Finds the bounding box of a PointCloud. If no PointCloud is
#         passed then it will use self.try_to_detect() to find an object.

#         The resulting FindClusterBoundingBoxResponse msg is stored in
#         self.last_box_msg.

#         Example usage:
#         >> res = detector.detect_narrow()
#         #check res is not None
#         >> cluster = detector.find_biggest_cluster(res.detection.clusters)
#         #any other cluster selector is ok
#         >> box_msg = detector.detect_bounding_box(cluster)

#         Parameters:
#         cluster: a PointCloud or PointCloud2 msg. It can be returned by one of the detect_*
#                  methods. If none an object will be searched for.
#         cluster_choser: if cluster is None, use this choser to select when
#                         detecting. Default to find_random_cluster.

#         Return:
#         a FindClusterBoundingBoxResponse msg.
#         """

#         try:
#             finder = self.__getattribute__(cluster_choser)
#         except AttributeError:
#             rospy.logwarn("Cluster choser %s does not exist, using the random one" %
#                           cluster_choser)
#             finder = self.find_random_cluster

#         if cluster is None:
#             detection_result = self.try_to_detect()
#             if detection_result is None:
#                 rospy.logerr("No way there is an object in front of me!")
#                 self.last_box_msg = None
#                 return None
#             cluster = finder(detection_result.detection.clusters)

#         req = FindClusterBoundingBoxRequest()
#         #using only PointCloud2 now
#         if type(cluster) is PointCloud:
#             cluster = PointCloud_to_PointCloud2(cluster)
#         req.cluster = cluster
#         try:
#             self.last_box_msg = self.box_detector(req)
#         except rospy.ServiceException, e:
#             rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)
#             return None
#         if not self.last_box_msg.error_code:
#             return self.last_box_msg
#         else:
#             rospy.logwarn("An error was reported when trying tabletop")
#             return self.last_box_msg

#     def point_head_at(self, mover, box_msg = None,
#                       cluster_choser = "find_random_cluster"):
#         """Points the head towards an object, represented by the
#         object_manipulation_msgs/FindClusterBoundingBoxResponse msg. If box_msg
#         is None then it will invoke try_to_detect().

#         Parameters:
#         mover: a Pr2JointMover instance.
#         box_msg: A FindClusterBoundingBoxResponse msg, or None.
#         cluster_choser: the name of the method to use to select the cluster.
#             Default to find_random_cluster.

#         Returns:
#         True if it could find the object or box_msg is not None, False otherwise.
#         """

#         if box_msg is None:
#             res = self.try_to_detect()
#             if res is None:
#                 rospy.logerr("No object found!")
#                 return False
#             clusters = res.detection.clusters

#             try:
#                 finder = self.__getattribute__(cluster_choser)
#             except AttributeError:
#                 rospy.logwarn("Cluster choser %s does not exist, using the random one" %
#                               cluster_choser)
#                 finder = self.find_random_cluster

#             object_cluster = finder(clusters)
#             box_msg = self.detect_bounding_box(cluster = object_cluster)

#             if box_msg is None: #still!!
#                 return False
#         position = (box_msg.pose.pose.position.x,
#                     box_msg.pose.pose.position.y,
#                     box_msg.pose.pose.position.z)
#         frame = box_msg.pose.header.frame_id
#         mover.point_head_to(position, frame)
#         return True

#     def draw_bounding_box(self, id, box_msg, color = (1.0, 1.0, 0.0, 0.0)):
#         marker = Marker()
#         marker.header.stamp = rospy.Time.now()
#         marker.ns = "object_detector"
#         marker.type = Marker.CUBE
#         marker.action = marker.ADD
#         marker.id = id
#         marker.header.frame_id = box_msg.pose.header.frame_id

#         marker.pose = box_msg.pose.pose
#         marker.scale.x = box_msg.box_dims.x
#         marker.scale.y = box_msg.box_dims.y
#         marker.scale.z = box_msg.box_dims.z

#         marker.color.a = color[0]
#         marker.color.r = color[1]
#         marker.color.g = color[2]
#         marker.color.b = color[3]

#         self.box_drawer.publish(marker)

#     def search_for_object(self, mover, trials = 1,
#                           cluster_choser="find_random_cluster",
#                           max_pan=0.4, min_pan=-0.4,
#                           max_tilt = 1.1, min_tilt = 0.8):

#         """

#         Returns:
#         True if successfull, False otherwise
#         """
#         #first try without moving
#         if self.point_head_at(mover,cluster_choser = cluster_choser):
#             return True
#         trials -= 1
#         while trials > 0:
#             pan = random.uniform(min_pan, max_pan)
#             tilt = random.uniform(min_tilt, max_tilt)
#             mover.set_head_state((pan, tilt))
#             rospy.sleep(0.5)

#             if self.point_head_at(mover,cluster_choser = cluster_choser):
#                 return True
#             trials -= 1
#         return False


class GenericDetector(object):
    """An interface to the tabletop detection and segmentation provided by ROS.
    """
    def __init__(self,
                 detector_service="object_detection",
                 box_detector="find_cluster_bounding_box2",
                 collision_processing = "/tabletop_collision_map_processing/tabletop_collision_map_processing",
                 tabletop_segmentation = "/tabletop_segmentation"):

        if detector_service is not None:
            rospy.loginfo("waiting for %s service" % detector_service)
            rospy.wait_for_service(detector_service)
            self.detector =  rospy.ServiceProxy(detector_service,
                    TabletopDetection)
        else:
            rospy.logwarn("Not using any object detector service")
            self.detector = None

        if box_detector is not None:
            rospy.loginfo("waiting for %s service" % box_detector)
            rospy.wait_for_service(box_detector)
            self.box_detector =  rospy.ServiceProxy(box_detector,
                    FindClusterBoundingBox2)
        else:
            rospy.loginfo("Not using any box detector service")
            self.box_detector = None


        if collision_processing is not None:
            rospy.loginfo("Waiting for collision processing service %s to come up",
                          collision_processing)
            rospy.wait_for_service(collision_processing)
            self.collision_processing = rospy.ServiceProxy(collision_processing,
                    TabletopCollisionMapProcessing)
        else:
            self.collision_processing = None
        

        if tabletop_segmentation is not None:
            rospy.loginfo("Waiting for segmentation service %s to come up", tabletop_segmentation)
            rospy.wait_for_service(tabletop_segmentation)
            self.segment_only_srv = rospy.ServiceProxy(tabletop_segmentation,
                                                   TabletopSegmentation)
        else:
            self.segment_only_srv = None


        self.box_drawer = rospy.Publisher("visualization_marker",
                                          Marker
                                          )
        self.last_detection_msg = None
        self.last_box_msg = None
        self.last_collision_processing_msg = None
        self.last_segmentation_msg = None

        rospy.loginfo("TabletopDetector is ready")

    def __table_test(self, detection_msg):
        table = detection_msg.detection.table
        table_quat = (table.pose.pose.orientation.x,
                      table.pose.pose.orientation.y,
                      table.pose.pose.orientation.z,
                      table.pose.pose.orientation.w)
        table_angles = transformations.euler_from_quaternion(table_quat)
        if (math.fabs(table_angles[0]) < 0.1 and
            math.fabs(table_angles[1]) < 0.1):
            return True
        else:
            rospy.logwarn("Table has wrong angles: %s",str(table_angles))
            return False

    def __detect(self, detector):
        req = TabletopDetectionRequest()
        req.return_clusters = True
        req.return_models = True
        try:
            reply = detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling object_detection: %s"%e)
            return None
        if reply.detection.result != reply.detection.SUCCESS:
            return None
        if len(reply.detection.clusters) == 0:
            return None
        if not self.__table_test(reply):
            return None
        return reply

    def detect(self):
        """Detects an object.
        If an object is found it returns the resulting
        tabletop_object_detector/TabletopDetectionResponse msg,
        None otherwise
        """
        self.last_detection_msg = self.__detect(self.detector)
        if self.last_detection_msg is None:
            rospy.logwarn("No detection done!!")
        return self.last_detection_msg

    def segment_only(self):
        """Calls the tabletop segmentation service. The difference with detect is
        that no object recognition is performed.

        Returns None if no service is specified or if no object is segmented,
        a tabletop_object_detector/TabletopDetectionResponse otherwise.
        """
        if self.segment_only_srv is None:
            rospy.logerr("No segmentation service specified!")
            return None

        res = self.segment_only_srv()
        isinstance(res, TabletopSegmentationResponse)
        if res.result != res.SUCCESS:
            rospy.logwarn("Segmentation did not work, error is %d", res.result)
            return None

        self.last_segmentation_msg = res

        detection_msg = TabletopDetectionResponse()
        num_objects = len(res.clusters)
        detection_msg.detection.table = res.table
        detection_msg.detection.clusters = res.clusters
        detection_msg.detection.result = res.SUCCESS
        detection_msg.detection.models = [DatabaseModelPoseList() for i in range(num_objects)]
        detection_msg.detection.cluster_model_indices = range(num_objects)
        self.last_detection_msg = detection_msg

        return detection_msg

    def find_biggest_cluster(self, clusters, return_index=False):
        '''Select the biggest cluster among clusters. It uses
        detect_bounding_box to find the cluster position

        Params:
        clusters: a list of PointCloud among wich to find the closest
        cluster.
        return_index: boolean for whether or not return the index of the chosen cluster along with the cluster itself

        A tipical usage is to call res = detect_[narrow|wide], and then select
        the cluster using find_biggest_cluster(res.detection.clusters)
        '''
        if clusters is None:
            return None

        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return
        #shortcut for a single object
        if len(clusters) == 1:
            return clusters[0]

        #finding the biggest cluster
        #TODO use the builtin max function
        max_len = 0
        index = 0
        for i,c in enumerate(clusters):
            if len(c.points) > max_len:
                max_len = len(c.points)
                index = i
        object_cluster = clusters[index]
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points)))

        if return_index:
            return object_cluster, index
        else:
            return object_cluster

    def find_random_cluster(self, clusters):
        '''Selects a random cluster.

        Parameters:
        clusters: a list of PointCloud among wich to find the closest
        cluster.

        A tipical usage is to call res = detect_[narrow|wide], and then select
        the cluster using find_random_cluster(res.detection.clusters)
        '''
        if clusters is None:
            return None

        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return None

        #using a random cluster
        index = random.randint(0, len(clusters)-1)
        object_cluster = clusters[index]
        rospy.loginfo("Using object %d with %d points"%(index, len(object_cluster.points)))
        return object_cluster

    def find_closest_cluster(self, clusters):
        '''Searches for the closest cluster among clusters. It uses
        detect_bounding_box to find the cluster position

        Parameters:
        clusters: a list of PointCloud among wich to find the closest
        cluster.

        A tipical usage is to call res = detect, and then select
        the cluster using find_closest_cluster(res.detection.clusters)
        '''
        if clusters is None:
            return None

        if len(clusters) == 0:
            rospy.logerr("No object found!")
            return None

        boxes = []
        for cluster in clusters:
            box_msg = self.detect_bounding_box(cluster)
            if box_msg is not None:
                boxes.append(box_msg)

        if len(boxes) == 0:
            return None

        closest_index = 0
        closest_dist = 10000

        for i, box in enumerate(boxes):
            dist = math.sqrt(box.pose.pose.position.x**2 +
                             box.pose.pose.position.y**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        rospy.loginfo("Using object %d with %f distance"%
                      (closest_index, closest_dist))
        return clusters[closest_index]


    def call_collision_map_processing(self, detection_result,
                                      reset_collision_models = True,
                                      reset_attached_models = True,
                                      desired_frame = "/base_link"):
        """Calls the collision map processing service. Given the result of
        detect, adds the objects to the collision map and returns them as
        instances of GraspableObjects with correct collision names.

        Parameters:
        detection_result: a tabletop_object_detector/TabletopDetectionResponse msg,
          usually returned by detect().
        reset_collision_models: whether the current collision models should be reset before adding new models
        reset_attached_models: whether the current list of models attached to the robot should be reset
        desired_frame: what tf frame the results should be returned in
        """


        if detection_result is None:
            rospy.logerr("Error: using a None detection_result")
            return None

        rospy.loginfo("Calling collision map processing")
        processing_call = TabletopCollisionMapProcessingRequest()
        processing_call.detection_result = detection_result.detection

        processing_call.reset_attached_models = reset_attached_models
        processing_call.reset_collision_models = reset_collision_models
        processing_call.desired_frame = desired_frame

        try:
            self.last_collision_processing_msg = self.collision_processing.call(processing_call)
        except ServiceException, e:
            rospy.logerr("Error calling collision map: %s" % str(e))
            self.last_collision_processing_msg = None
        return self.last_collision_processing_msg

    def get_min_max_box(self, box, padding = 0):
        """
        Returns the minimum and the maximum points
        of a bounding box.
        Note: Using only two points to describe a cube assumes that the cube
        is parallel to the frame of reference defined in box. If not it will be
        an approximation.
        
        Parameters:
        box: a FindClusterBoundingBoxResponse msg
        padding: a float to increase the dimensions of the box
        
        Returns:
        (xmin, ymin, zmin), (xmax, ymax, zmax)
        """
        assert isinstance(box, FindClusterBoundingBox2Response)
        #translation = [box.pose.pose.position.x,
                       #box.pose.pose.position.y,
                       #box.pose.pose.position.z,
                       #1
                      #]
        #rotation = [box.pose.pose.orientation.x,
                    #box.pose.pose.orientation.y,
                    #box.pose.pose.orientation.z,
                    #box.pose.pose.orientation.w
                   #]        
        #angles = transformations.euler_from_quaternion(rotation)
        #T = transformations.compose_matrix(translate=translation,
                                           #angles=angles)
        x_dim = box.box_dims.x + padding
        y_dim = box.box_dims.y + padding
        z_dim = box.box_dims.z + padding

        position = [box.pose.pose.position.x,
                    box.pose.pose.position.y,
                    box.pose.pose.position.z,
                    ]

        
        pmin = (position[0] - x_dim/2., 
                position[1] - y_dim/2., 
                position[2] - z_dim/2)
        
        pmax = (position[0] + x_dim/2., 
                position[1] + y_dim/2., 
                position[2] + z_dim/2)
        
        return pmin, pmax         
        

    def detect_bounding_box(self, cluster = None,
                            cluster_choser = None):
        """Finds the bounding box of a PointCloud. If no PointCloud is
        passed then it will use self.detect() to find an object.

        The resulting FindClusterBoundingBoxResponse msg is stored in
        self.last_box_msg.

        Example usage:
        >> res = detector.detect()
        #check res is not None
        >> cluster = detector.find_biggest_cluster(res.detection.clusters)
        #any other cluster selector is ok
        >> box_msg = detector.detect_bounding_box(cluster)

        Parameters:
        cluster: a PointCloud or Pointcloud2 msg. It can be returned by one the detect
                 methods. If none an object will be searched for.
        cluster_choser: if cluster is None, use this choser to select when
                        detecting. This has to be a function that takes a
                        list pf PointCloud msg and returns one of them.
                        Examples are in self.find_* functions.
                        Default to self.find_random_cluster.

        Return:
        a FindClusterBoundingBoxResponse msg, or None if an error occured.
        """

        if cluster_choser is None:
            cluster_choser = self.find_random_cluster
        finder = cluster_choser

        if cluster is None:
            detection_result = self.detect()
            if detection_result is None:
                rospy.logerr("No way there is an object in front of me!")
                self.last_box_msg = None
                return None
            cluster = finder(detection_result.detection.clusters)

        if type(cluster) is PointCloud:
            cluster = PointCloud_to_PointCloud2(cluster)
          
        if type(cluster) is not PointCloud2:
            rospy.logerr("cluster has to be a pointcloud2, while it's a %s",
                         type(cluster))
            return None
        
        req = FindClusterBoundingBox2Request()
        req.cluster = cluster
        
        try:
            self.last_box_msg = self.box_detector(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)
            return None
        if not self.last_box_msg.error_code:
            return self.last_box_msg
        else:
            rospy.logwarn("An error was reported when trying tabletop")
            return self.last_box_msg

    def point_head_at(self, mover, box_msg = None,
                      cluster_choser = None):
        """Points the head towards an object, represented by the
        object_manipulation_msgs/FindClusterBoundingBoxResponse msg. If box_msg
        is None then it will invoke detect().

        Parameters:
        mover: a Pr2JointMover instance.
        box_msg: A FindClusterBoundingBoxResponse msg, or None.
        cluster_choser: the name of the method to use to select the cluster.
            Default to find_random_cluster.

        Returns:
        True if it could find the object or box_msg is not None, False otherwise.
        """

        if box_msg is None:
            res = self.detect()
            if res is None:
                rospy.logerr("No object found!")
                return False
            clusters = res.detection.clusters

            if cluster_choser is None:
                finder = self.find_random_cluster
            else:
                finder = cluster_choser


            object_cluster = finder(clusters)
            box_msg = self.detect_bounding_box(cluster = object_cluster)

            if box_msg is None: #still!!
                return False
        position = (box_msg.pose.pose.position.x,
                    box_msg.pose.pose.position.y,
                    box_msg.pose.pose.position.z)
        frame = box_msg.pose.header.frame_id
        mover.point_head_to(position, frame)
        return True

    def draw_bounding_box(self, id, box_msg, color = (1.0, 1.0, 0.0, 0.0),
                          duration = 0.0):
        """
        Draws a bounding box as detectd by detect_bounding_box.
        
        Parameters: 
        box_msg is a FindClusterBoundingBoxResponse msg.
        color: a quadruple with alpha, r,g,b
        duration: how long should the bounding box last. 0 means forever.
        """
        
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_detector"
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.header.frame_id = box_msg.pose.header.frame_id

        marker.pose = box_msg.pose.pose
        marker.scale.x = box_msg.box_dims.x
        marker.scale.y = box_msg.box_dims.y
        marker.scale.z = box_msg.box_dims.z

        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]
        marker.lifetime = rospy.Duration(duration)

        self.box_drawer.publish(marker)
 


    def draw_box_marker(self, id,  x, y, z, dimx, dimy, dimz,
         color = (1.0,1.0,1.0,0.0),duration=0.0, frame_id="base_link"):
                
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_detector"
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.header.frame_id = "base_link"

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.position.x = x
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = dimx
        marker.scale.y = dimy
        marker.scale.z = dimz

        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]
        marker.lifetime = rospy.Duration(duration)

        self.box_drawer.publish(marker)



    def draw_table_rviz(self, id, table_msg, color = (1.0, 1.0, 1.0, 0.0), duration = 0.0):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object_detector"
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.header.frame_id = table_msg.pose.header.frame_id

        marker.pose = table_msg.pose.pose
        marker.pose.position.x = (table_msg.x_max-table_msg.x_min)/2 \
            + table_msg.x_min
        marker.pose.position.y = (table_msg.y_max-table_msg.y_min)/2 \
            + table_msg.y_min

        marker.scale.x = table_msg.x_max - table_msg.x_min
        marker.scale.y = table_msg.y_max - table_msg.y_min
        marker.scale.z = 0.01

        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]
        marker.lifetime = rospy.Duration(duration)

        self.box_drawer.publish(marker)
        

    def search_for_object(self, mover, trials = 1,
                          cluster_choser=None,
                          max_pan=0.4, min_pan=-0.4,
                          max_tilt = 1.1, min_tilt = 0.8):

        """

        Returns:
        True if successfull, False otherwise
        """
        #first try without moving
        if self.point_head_at(mover,cluster_choser = cluster_choser):
            return True
        trials -= 1
        while trials > 0:
            pan = random.uniform(min_pan, max_pan)
            tilt = random.uniform(min_tilt, max_tilt)
            mover.set_head_state((pan, tilt))
            rospy.sleep(0.5)

            if self.point_head_at(mover,cluster_choser = cluster_choser):
                return True
            trials -= 1
        return False
    
    
    def find_all_clusters(self, trials, draw_markers = True):
        '''Find all clusters with current head position. Returns list of 
            cluster messages and a table message.
            
            Performs zero head movement.
            '''
   	
        clusters = []
        table_detected = False
        table = None

        #rospy.sleep(15)
        print "\n\n\n\n\nstarting "

    	while trials > 0:
            res = self.detect() 
            if res is not None:
                print("Detected clusters, length: " + str(len(res.detection.clusters)))
                clusters += res.detection.clusters

                if not table_detected:
                    table = res.detection.table
                    table_detected = True
            else:
                print("No clusters detected.")

            trials -= 1

        if draw_markers:
            # Draw markers for rviz:
            for i in range(len(clusters)):
                print "drawing box " +  repr(i)
                self.draw_bounding_box(i, self.detect_bounding_box(clusters[i]))

            self.draw_table_rviz(100, table)

            self.draw_box_marker(200, 1,1,1, 0.01,0.01,0.3)
            self.draw_box_marker(201, 1,1,1, 0.3,0.01,0.01)

        return clusters, table        
