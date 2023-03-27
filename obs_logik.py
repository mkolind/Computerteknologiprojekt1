while not rospy.is_shutdown():
            lightData = lightSensor.getColour()
            rospy.loginfo(lightData)

            lidar_distances = self.get_scan(180)
            front_cone = lidar_distances[67:113] # 45 grader
            left_cone = lidar_distances[0:67] # 67 grader
            right_cone = lidar_distances[113:180] # sjovt nok ogs 67


            if turtlebot_moving:
                min_distance = min(front_cone)
                if min_distance < SAFE_STOP_DISTANCE:
                    twist.linear.x = 0
                    turtlebot_moving = False
                else:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    turtlebot_moving = True


            else:
                min_distance = min(lidar_distances)
                if min(front_cone) > SAFE_STOP_DISTANCE*2:
                    twist.linear.x = LINEAR_VEL
                    turtlebot_moving = True
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('Distance of the obstacle : %f', min_distance)
                    rospy.loginfo('Distance of the obstacle : %f', min_distance)
                    continue


                min_index = lidar_distances.index(min_distance)
                if min_distance < SAFE_STOP_DISTANCE*1.5:
                    if(min_index >= 90):
                        factor = -1
                    else:
                        factor = 1

                    twist.linear.x = 0
                    twist.angular.z = 1*factor
                    turtlebot_moving = False
                    rospy.loginfo('Turning')
                else:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    turtlebot_moving = True

            self._cmd_pub.publish(twist)
            rospy.loginfo('Distance of the obstacle : %f', min_distance)
