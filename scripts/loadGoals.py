from pymongo import MongoClient
import json

import logging
import random

if __name__ == '__main__':

    # set logging level
    logging.getLogger().setLevel(logging.DEBUG)

    # connect to local mongodb server
    client = MongoClient()
    # get IROS23 db
    db = client.iros23
    # get MAP1 collection
    map = db.map1
    # clear collection
    map.drop()

    with open('./map1.json') as file:
      file_data = json.load(file)

    map.insert_many(file_data)
