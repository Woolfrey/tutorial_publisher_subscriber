# ROS2 Tutorial : Publisher & Subscriber

## Contents

## What are they?

A ROS `publisher` is analogous to a newsagency. A literary publisher will print and distribute newspapers or magazines at regular intervals (daily, weekly, monthly) and make them publically available at places like a newsagent. Multiple people are then free to purchase or subscribe to the media that is relevant to their interests.

Similarly, a `publisher` node generates information and makes it publically available on the ROS network. A `subscriber` node can then obtain this information for its own use. Examples include:
- Sensor information:
     - Joint positions
     - Laser scans
     - Depth sensor
- Location information:
     - Robot pose
     - GPS location

It is suitable for any kind of information that is generated frequently with little or no computational requirements.

In contrast, information that is required infrequently, and that requires some more complex transformation of data might be better distributed using a `Client` and `Server` paradigm. For example:
- Acquiring map updates
- Generating a new path plan

ROS has 3 types of communication paradigms:

| Sender        | Receiver      | Messaging Frequency |
|---------------|---------------|---------------------|
| Publisher     | Subscriber    | Continuous          |
| Server        | Client        | Sporadic            |
| Action Server | Action Client | Both                |

## Writing a Publisher

## Writing a Subscriber
