/* This object is an abstract layer on the specificities of 
   a robot humanoid

   Copyright (c) 2005-2006, 
   Francois Keith,
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please refers to file License.txt for details on the license.
*/
#ifndef _JRLCIRDYNAMICS_HUMANOID_SPECIFICITIES_JRL_JAPAN_H_
#define _JRLCIRDYNAMICS_HUMANOID_SPECIFICITIES_JRL_JAPAN_H_
#include <vector>
#include "dynamics-config.h"
#include "dynamicsJRLJapan/DynamicMultiBody.h"

namespace dynamicsJRLJapan
{
    
  /*@! This Object here implements the constants 
    and the specificities of HRP-2.
    */
  class DYN_JRL_JAPAN_EXPORT HumanoidSpecificities
  {
  public:

    // Constructor
    HumanoidSpecificities();

    // Destructor
    ~HumanoidSpecificities();
    
    // Returns the width of the foot given in parameter:
    // @param WhichFoot : -1 Right foot 1 Left foot.
    // @paran Depth: depth of the foot (X)
    // @param Width: width of the foot (Y), 
    // @param Height: height of the foot (Z),
    // @param 
    // @return -1 if an error occured,
    //  0 otherwise.
    int GetFootSize(int WhichFoot, double &Depth, double &Width,double &Height);
    
    // Returns the width of the foot given in parameter:
    // @param aFileName : Name of the file where the humanoid parameters are stored.
    // @paran HumanoidName : Name of the humanoid. 
    int ReadXML(string & aFileName, string & HumanoidName);

    // Display the information stored inside the object.
    // Namely for debugging.
    void Display();

    // Returns the length of the tibia
    // @param WhichSide: -1 Right 1 Left.
    double GetTibiaLength(int WhichSide);

    // Returns the length of the femur
    // @param WhichSide: -1 Right 1 Left.
    double GetFemurLength(int WhichSide);

    // Returns the length of the Upper arm
    // @param WhichSide: -1 Right 1 Left.
    double GetUpperArmLength(int WhichSide);

    // Returns the length of the Fore arm
    // @param WhichSide: -1 Right 1 Left.    
    double GetForeArmLength(int WhichSide);

    // Returns the ankle position
    // @param WhichSide: -1 Right 1 Left.    
    // @return AnklePosition: (X,Y,Z)
    void GetAnklePosition(int WhichSide, double AnklePosition[3]);

    // Returns the position of the Hip regarding the waist's origin.
    // @param WhichSide: -1 Right 1 Left.
    // @ return WaistToHip translation.
    void GetWaistToHip(int WhichSide, double WaistToHip[3]);
    
    // Returns the Hip's length, for instance in HRP-2 the Y-axis
    // for the hip is translated regarding the X and Z axis.
    // @param WhichSide: -1 Right 1 Left.
    // @ return Hip lenght.
    void GetHipLength(int WhichSide,double HipLength[3]);

    /*! \name Joints related methods 
      @{
     */
    
    // Returns the number of joints for the arms */
    int GetArmJointNb(int WhichSide);

    // Returns the joints for one arm */
    const std::vector<int> & GetArmJoints(int WhichSide);

    // Returns the number of joints one leg */
    int GetLegJointNb(int WhichSide);

    // Returns the joints for one leg */
    const std::vector<int> & GetLegJoints(int WhichSide);

    // Returns the number of joints for one foot */
    int GetFootJointNb(int WhichSide);

    // Returns the joints for one foot */
    const std::vector<int> & GetFootJoints(int WhichSide);
    
    
    // Returns the number of joints for the head */
    int GetHeadJointNb();

    // Returns the joints for the head*/
    const std::vector<int> & GetHeadJoints();
    
   // Returns the number of joints for the Chest */
    int GetChestJointNb();

    // Returns the joints for the Chest*/
    const std::vector<int> & GetChestJoints();
 
    // Returns the number of joints for the Upper Body.
    int GetUpperBodyJointNb();

    // Returns the vector of joints index for the
    // Upper body.
    const std::vector<int> & GetUpperBodyJoints();

    // Returns the number of joints for the Waist.
    int GetWaistJointNb();

    /*! \brief Returns the vector of joints index for the
      waist. */
    const std::vector<int> & GetWaistJoints();

    /*! @} */
  private:

    /*! Init the upper body joints. */
    int InitUpperBodyJoints();
    
    /*! \brief  Store foot's height, width and depth. */
    double m_FootHeight[2]; // (Z)
    double m_FootWidth[2]; // (Y)
    double m_FootDepth[2]; // (X)

    /*! \brief Number of joints for each feet */
    int m_FeetJointNb[2];

    /*! \brief Vector of joints for each feet */
    std::vector<int> m_FeetJoints[2];

    /*! \brief Store position of the ankles in the feet. */
    double m_AnklePosition[2][3];
    
    /*! \brief Tibia's length */
    double m_TibiaLength[2];
    
    /*! \brief Femur's length */
    double m_FemurLength[2];

    /*! \brief Upper arm's length. */
    double m_UpperArmLength[2];

    /*! \brief Forearm's length. */
    double m_ForeArmLength[2];

    /*! \brief Waist to hip translation */
    double m_WaistToHip[2][3];

    /*! \brief Hip length */
    double m_HipLength[2][3];

    /*! \brief Arms number of joints. */
    int m_ArmsJointNb[2];

    /*! \brief Arms Joints. */
    std::vector<int> m_ArmsJoints[2];

    /*! \brief Legs number of joints. */
    int m_LegsJointNb[2];

    /*! \brief Legs Joints. */
    std::vector<int> m_LegsJoints[2];

    /*! \brief Number of head joints */
    int m_HeadJointNb;

    /*! \brief Head joints */
    std::vector<int> m_HeadJoints;

    /*! \brief Number of Chest joints */
    int m_ChestJointNb;

    /*! \brief Chest joints */
    std::vector<int> m_ChestJoints;

    /*! \brief Number of upper body joints.
      The algorithm is quite simple:
      m_HeadJointNb + m_ChestJointNb + m_ArmJointNb[0] 
      + m_ArmJointNb[1]
     */
    int m_UpperBodyJointNb;

    /*! \brief List of upper body joints.
      The joint which follows the above algorith.
     */
    std::vector<int> m_UpperBodyJoints;

    /*! \brief Number of waist joint. */
    int m_WaistJointNb;
    
    /*! \brief List of waist joints. */
    std::vector<int> m_WaistJoints;

  };

};

#endif
