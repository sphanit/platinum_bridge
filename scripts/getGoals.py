#!/usr/bin/env python
from pymongo import MongoClient
import rospy
from platinum_bridge.srv import getGoal, getGoalResponse


# connect to local mongodb server
client = MongoClient()
# get IROS23 db
db = client.iros23
# get MAP1 collection
map = db.map1

def sendGoal(req):
  query = {"label": req.goal_name}
  for doc in map.find(query):
      print("> {}".format(doc))
      return getGoalResponse(doc['coordinates'])

if __name__ == '__main__':
    rospy.init_node('mongodb_goals')
    s = rospy.Service('~get_goal', getGoal, sendGoal)
    rospy.spin()
