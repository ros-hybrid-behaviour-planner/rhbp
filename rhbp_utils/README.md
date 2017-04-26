The package rhbp_utils provides additional components, which are not in the core of rhbp.
# Knowledge Sensors
The package provides two sensors, for integrating the Knowledge Base into rhbp. The details of using a sensor in rhbp, can be found in the documentation of rhbp_core.
## Knowledge Sensor
The knowledge sensor provides a boolean value, whether any matching fact exists. Therefore it takes a pattern at instantiation and subscribes for updates on the knowledge base.
An example for the usage can be found in the file knowledge_sensor_test.py
## Knowledge Fact Sensor
The KnowledgeFactSensor takes a pattern and provides a list of all matching facts.