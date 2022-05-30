#!/usr/bin/env python3
"""
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


import rospy
import mqtt_scene_integration.msg

import object_loader_msgs.srv
import object_loader_msgs.msg

add_obj=[]
del_obj=[]

fixture_slot=dict()
def fixture_callback(data,position):
    global fixture_slot
    global add_obj
    global del_obj
    rospy.loginfo("position %f",position)
    print(data)

    if (data.state=="Empty"):
        obj_req=object_loader_msgs.srv.RemoveObjectsGroupRequest()
        obj_req.objects_group=fixture_slot[position]["name"]+"_"+fixture_slot[position]["content"]
        obj_res=del_obj(obj_req)
        fixture_slot[position]["content"]=""
    else:
        fixture_slot[position]["content"]=data.content
        obj_req=object_loader_msgs.srv.AddObjectsGroupRequest()
        obj_req.objects_group=fixture_slot[position]["name"]+"_"+fixture_slot[position]["content"]
        obj_res=add_obj(obj_req)

def main():
    global fixture_slot
    global add_obj
    global del_obj
    rospy.init_node('node')

    add_obj = rospy.ServiceProxy('/add_objects_group', object_loader_msgs.srv.AddObjectsGroup)
    del_obj = rospy.ServiceProxy('/remove_objects_group', object_loader_msgs.srv.RemoveObjectsGroup)

    positions=range(0,4)
    for p in positions:
        fixture_slot[p]={"content": "",
                         "name": "P"+str(p)}
        print(fixture_slot)
        rospy.Subscriber("JFMX/L1/sharework/station/p"+str(p), mqtt_scene_integration.msg.Fixture, fixture_callback,p)

    rospy.spin()


#JFMX/L1/sharework/station/p{0
if __name__ == '__main__':
    main()
