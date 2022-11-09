import xml.etree.cElementTree as ET
import re
import yaml
import os

class YAMLtoMuJoCo():
    
    def __init__(self, yaml_file):
        with open(yaml_file, 'r') as file:
            self.data = yaml.safe_load(file)
        self.default_color = "0.59 0.44 0.50 1"
        self.default_target_color = "0.2 0.939216 0.239216 1"
        self.default_obstacle_color = "0.59 0.44 0.50 1"
        

    def createXML(self, output_file):
        self.root = ET.Element("mujoco", model="environment")
        target = self.getTarget()
        self.setTarget(target)
        obstacles = self.getObstacles()
        for obstacle in obstacles:
            print("WE HERE")
            if obstacles[obstacle]["type"] == "sphere":
                obj = ET.SubElement(self.root, "body", name=obstacle, pos="%s" % obstacles[obstacle]["position"])
                ET.SubElement(obj, "geom", type=obstacles[obstacle]["type"], size="%s" % obstacles[obstacle]["size"], rgba=obstacles[obstacle]["rgba"])
            if obstacles[obstacle]["type"] == "cylinder":
                target_object = ET.SubElement(self.root, "body", name=obstacle)
                ET.SubElement(target_object, "geom", type=obstacles[obstacle]["type"], fromto="%s %s" % (obstacles[obstacle]["from"], obstacles[obstacle]["to"]), size="%s" % obstacles[obstacle]["size"], rgba=obstacles[obstacle]["rgba"])
        tree = ET.ElementTree(self.root)
        tree.write(output_file)
        print("Wrote to ", output_file)

    
    def getTarget(self):
        print("Getting Target")
        target = self.data["target"]
        try:
            rgba_val = target["rgba"]
        except:
            target["rgba"] = self.default_target_color
            
        return target

    def getObstacles(self):
        obstacles = {}
        print("Getting Obs")
        
        for item in self.data:
            if "obstacle" in item:
                obstacles[item] = self.data[item]
                try:
                    rgba_val = obstacles[item]["rgba"]
                except:
                    obstacles[item]["rgba"] = self.default_obstacle_color
                    
        return obstacles
    
    def setTarget(self, target):
        target_object = ET.SubElement(self.root, "body", name="target", pos="%s" % target["position"])
        ET.SubElement(target_object, "geom", type=target["type"], contype="0", conaffinity="0", size="%s" % target["size"], rgba=target["rgba"])
        

# n = YAMLtoMuJoCo("plan.yaml")
# os.chdir("../mjc")

# n.createXML("problems/obs.xml")

# n.createXML("obs.xml")

