/** \page pbuildingarobot Building a robot
    
    \section secobjectfactory Object Factory

    A robot model is build using the following components:
    <ul>
      <li> Bodies</li>
      <li> Joints</li>
      <li> a DynamicRobot </li>
    </ul>
    
    A humanoid robot is a particular robot which has
    in addition the following components:
    <ul>
      <li> Hand </li>
      <li> Foot </li>
      <li> a HumanoidRobot </li>
    </ul>

    All those components are created using an object factory.
    dynamicsJRLJapan provides one of such object factory with the
    class ObjectFactory.<br>
    Creating an empty  humanoid robot is therefore very simple:
    <code>

    dynamicsJRLJapan::ObjectFactory robotDynamicsObjectConstructor; 

    CjrlHumanoidDynamicRobot * aHDR = robotDynamicsObjectConstructor.createHumanoidDynamicRobot();
    </code>

    
 */

