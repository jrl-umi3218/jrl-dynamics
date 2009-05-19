#include <string>
#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
#include "jrlMathTools/jrlConstants.h"
#include <GL/glut.h>

using namespace std;
using namespace dynamicsJRLJapan;

#define TRUE  1
#define FALSE 0

/* Dimensions of texture image. */
#define IMAGE_WIDTH  64
#define IMAGE_HEIGHT 64

/* Step to be taken for each rotation. */
#define ANGLE_STEP 10

//GLdouble view_h = 270, view_v = 0, head_angle = 0;
GLdouble view_h = 230, view_v = 270, head_angle = 0;

HumanoidDynamicMultiBody *aHDMB=0;

GLshort shaded = TRUE, anim = FALSE;
GLshort texture = FALSE, transparent = FALSE;
GLshort light1 = TRUE, light2 = FALSE;


GLubyte image[IMAGE_WIDTH][IMAGE_HEIGHT][3];

/* Variable used in the creaton of glu objects */
GLUquadricObj *obj;

/************ GLUT PART ****************/

/* Makes a simple check pattern image. (Copied from the redbook example
   "checker.c".) */
void
make_image(void)
{
  int i, j, c;

  for (i = 0; i < IMAGE_WIDTH; i++) {
    for (j = 0; j < IMAGE_HEIGHT; j++) {
      c = ((((i & 0x8) == 0) ^ ((j & 0x8)) == 0)) * 255;
      image[i][j][0] = (GLubyte) c;
      image[i][j][1] = (GLubyte) c;
      image[i][j][2] = (GLubyte) c;
    }
  }
}

/* Called when the model's window has been reshaped.  */
void 
myReshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(65.0, (GLfloat) w / (GLfloat) h, 1.0, 20.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -5.0);  /* viewing transform  */
  glScalef(4.0, 4.0, 4.0);
  //  glScalef(1.0, 1.0, 1.0);
}

/* ARGSUSED1 */
void
special(int key, int x, int y)
{
  switch (key) {
  case GLUT_KEY_LEFT:
    if ((view_h -= ANGLE_STEP) <= 0)
      view_h = 360;
    break;
  case GLUT_KEY_RIGHT:
    if ((view_h += ANGLE_STEP) >= 360)
      view_h = 0;
    break;
  case GLUT_KEY_UP:
    if ((view_v += ANGLE_STEP) >= 360)
      view_v = 0;
    break;
  case GLUT_KEY_DOWN:
    if ((view_v -= ANGLE_STEP) <= 0)
      view_v = 360;
    break;
  default:
    return;
  }
  cout << "view_h : " << view_h << " view_v: " << view_v << endl;
  glutPostRedisplay();
}


void DrawRecursiveJoint(CjrlJoint *aJoint)
{
  if (aJoint==0)
    return;
  
  Joint * a2Joint = dynamic_cast<Joint *>( aJoint);
  if (a2Joint==0)
    return;
  
  DynamicBody * aDB = (DynamicBody *)a2Joint->linkedBody();

  int NbChildren = aJoint->countChildJoints();

  
  for(int i=0;i<NbChildren;i++)
    {
      DynamicBody * lDB = (DynamicBody *)(((Joint *)a2Joint->childJoint(i))->linkedBody());
      glColor4f(1.0, 0.5, 0.9/(a2Joint->rankInConfiguration()+1), 1.0);

      double Ray=0.01;

      glBegin(GL_QUADS);
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)+Ray, aDB->p(2));
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)+Ray, aDB->p(2));
      glEnd();

      glBegin(GL_QUADS);
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(lDB->p(0)+Ray,lDB->p(1)-Ray, lDB->p(2));
      glVertex3f(lDB->p(0)-Ray,lDB->p(1)-Ray, lDB->p(2));
      glEnd();

      glBegin(GL_QUADS);
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)+Ray, aDB->p(2));
      glVertex3f(lDB->p(0)+Ray,lDB->p(1)+Ray, lDB->p(2));
      glVertex3f(lDB->p(0)+Ray,lDB->p(1)-Ray, lDB->p(2));
      glEnd();

      glBegin(GL_QUADS);
      glVertex3f(aDB->p(0)+Ray,aDB->p(1)+Ray, aDB->p(2));
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)+Ray, aDB->p(2));
      glVertex3f(lDB->p(0)-Ray,lDB->p(1)+Ray, lDB->p(2));
      glVertex3f(lDB->p(0)+Ray,lDB->p(1)+Ray, lDB->p(2));
      glEnd();

      glBegin(GL_QUADS);
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)+Ray, aDB->p(2));
      glVertex3f(aDB->p(0)-Ray,aDB->p(1)-Ray, aDB->p(2));
      glVertex3f(lDB->p(0)-Ray,lDB->p(1)-Ray, lDB->p(2));
      glVertex3f(lDB->p(0)-Ray,lDB->p(1)+Ray, lDB->p(2));
      glEnd();
      
    }
  
  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      DrawRecursiveJoint((CjrlJoint *)aJoint->childJoint(i)); 
    }

}

/* Called when a key is pressed. Checks if it reconises the key and if so
   acts on it, updateing the screen. */
/* ARGSUSED1 */
void
keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 's':
    if (shaded == FALSE) {
      cout << "LIGHTING" << endl;
      shaded = TRUE;
      glShadeModel(GL_SMOOTH);
      glEnable(GL_LIGHTING);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_COLOR_MATERIAL);
      gluQuadricNormals(obj, GLU_SMOOTH);
      gluQuadricDrawStyle(obj, GLU_FILL);
    } else {
      cout << "Flat Shade model"<< endl;
      shaded = FALSE;
      glShadeModel(GL_FLAT);
      glDisable(GL_LIGHTING);
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_COLOR_MATERIAL);
      gluQuadricNormals(obj, GLU_NONE);
      gluQuadricDrawStyle(obj, GLU_LINE);
      gluQuadricTexture(obj, GL_FALSE);
    }
    if (texture && !shaded);
    else
      break;
  case 't':
    if (texture == FALSE) {
      texture = TRUE;
      glEnable(GL_TEXTURE_2D);
      gluQuadricTexture(obj, GL_TRUE);
    } else {
      texture = FALSE;
      glDisable(GL_TEXTURE_2D);
      gluQuadricTexture(obj, GL_FALSE);
    }
    break;
  default:
    return;
  }
  glutPostRedisplay();
}

/* Called when a menu option has been selected. Translates the menu item
   identifier into a keystroke, then call's the keyboard function. */
void 
menu(int val)
{
  unsigned char key;

  switch (val) {
  case 1:
    key = 's';
    break;
  case 2:
    key = ' ';
    break;
  case 3:
    key = 't';
    break;
  case 4:
    key = 'o';
    break;
  case 5:
    key = '0';
    break;
  case 6:
    key = '1';
    break;
  case 7:
    key = '+';
    break;
  case 8:
    key = '-';
    break;
  default:
    return;
  }
  keyboard(key, 0, 0);
}

/* Initialises the menu of toggles. */
void 
create_menu(void)
{
  glutCreateMenu(menu);
  glutAttachMenu(GLUT_LEFT_BUTTON);
  glutAttachMenu(GLUT_RIGHT_BUTTON);
  glutAddMenuEntry("Shaded", 1);
  glutAddMenuEntry("Texture", 3);

}

void DrawLineRobot()
{
  /*
  glBegin(GL_QUADS);
  glColor4f(0.5,0.5,1.0,1.0);
  glVertex3f( -2.0, -2.0, 0.0);
  glVertex3f(  2.0, -2.0, 0.0);
  glVertex3f(  2.0,  2.0, 0.0);
  glVertex3f( -2.0,  2.0, 0.0);
  glEnd();
  */
  glDisable(GL_LIGHTING);
  DrawRecursiveJoint((Joint *)aHDMB->rootJoint());
}


/* Main display routine. Clears the drawing buffer and if transparency is
   set, displays the model twice, 1st time accepting those fragments with
   a ALPHA value of 1 only, then with DEPTH_BUFFER writing disabled for
   those with other values. */
void
display(void)
{
  int pass;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
    if (transparent) {
      glEnable(GL_ALPHA_TEST);
      pass = 2;
    } else {
      glDisable(GL_ALPHA_TEST);
      pass = 0;
    }

    /* Rotate the whole model */
    glRotatef(view_h, 0, 1, 0);
    glRotatef(view_v, 1, 0, 0);

    do {
      if (pass == 2) {
        glAlphaFunc(GL_EQUAL, 1);
        glDepthMask(GL_TRUE);
        pass--;
      } else if (pass != 0) {
        glAlphaFunc(GL_NOTEQUAL, 1);
        glDepthMask(GL_FALSE);
        pass--;
      }

      // Write drawing stuff here.
      DrawLineRobot();

    } while (pass > 0);
    glDepthMask(GL_TRUE);
    glutSwapBuffers();
  glPopMatrix();
}

/* Initialises texturing, lighting, display lists, and everything else 
   associated with the model. */
void 
myinit(void)
{
  GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat mat_shininess[] = {50.0};
  GLfloat light_position1[] = {1.0, 1.0, 1.0, 0.0};
  GLfloat light_position2[] = {-1.0, 1.0, 1.0, 0.0};

  glClearColor(128.0, 128.0, 255.0, 0.0);

  obj = gluNewQuadric();
  make_image();

  /* Set up Texturing */
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, IMAGE_WIDTH,
    IMAGE_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE,
    image);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  /* Set up Lighting */
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
  glLightfv(GL_LIGHT1, GL_POSITION, light_position2);

  /* Initial render mode is with full shading and LIGHT 0
     enabled. */
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_ALPHA_TEST);

  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glShadeModel(GL_SMOOTH);

  /* Initialise display lists */
  gluQuadricTexture(obj, GL_FALSE);
}

/************ TESTING PART *************/
void RecursiveDisplayOfJoints(CjrlJoint *aJoint)
{
  if (aJoint==0)
    return;

  Joint *a2Joint=0;

  int NbChildren = aJoint->countChildJoints();

  a2Joint = dynamic_cast<Joint *>( aJoint);
  if (a2Joint==0)
    return;

  cout << ((DynamicBody *)a2Joint->linkedBody())->p << endl;
  for(int i=0;i<NbChildren;i++)
    {
      // Returns a const so we have to force the casting/
      RecursiveDisplayOfJoints((CjrlJoint *)aJoint->childJoint(i)); 
    }
  //cout << " End for Joint: " << a2Joint->getName() << endl;
}


void DisplayDynamicRobotInformation(CjrlDynamicRobot *aDynamicRobot)
{
  std::vector<CjrlJoint *> aVec = aDynamicRobot->jointVector();
  int r = aVec.size();
  cout << "Number of joints :" << r << endl;
  for(int i=0;i<r;i++)
    {
      Joint * aJoint = dynamic_cast<Joint *>(aVec[i]);
      cout << aJoint->getName();
    }	

  
}

void DisplayMatrix(MAL_MATRIX(,double) &aJ)
{
  for(int i=0;i<6;i++)
    {
      for(unsigned int j=0;j<MAL_MATRIX_NB_COLS(aJ);j++)
	{
	  if (aJ(i,j)==0.0)
	    printf("0 ");
	  else
	    printf("%10.5f ",aJ(i,j));
	}
      printf("\n");
    }

}

void processMouse(int button, int state, int x, int y) 
{
  
  
}
void GoDownTree(const CjrlJoint * startJoint)
{
  std::cout << "joint ranked :" << startJoint->rankInConfiguration() << std::endl;
  std::cout << "Joint name :" << ((Joint *)startJoint)->getName() << std::endl;
  std::cout << "Mass of the body: " << ((DynamicBody *)(startJoint->linkedBody()))->getMasse() << std::endl;
  std::cout << "Name of the body: " << ((DynamicBody *)(startJoint->linkedBody()))->getName() << std::endl;

  std::cout << startJoint->currentTransformation() << std::endl;
  
  if (startJoint->countChildJoints()!=0)
    {
      const CjrlJoint * childJoint = startJoint->childJoint(0);
      GoDownTree(childJoint);
    }
}

int main(int argc, char *argv[])
{
  if (argc!=5)
    {
      cerr << " This program takes 4 arguments: " << endl;
      cerr << "./TestHumanoidDynamicRobotglut PATH_TO_VRML_FILE VRML_FILE_NAME "<< endl;
      cerr << " PATH_TO_SPECIFICITIES_XML PATH PATH_TO_MAP_JOINT_2_RANK" << endl;
      exit(-1);
    }	

  string aSpecificitiesFileName = argv[3];
  string aPath=argv[1];
  string aName=argv[2];
  string aMapFromJointToRank=argv[4];

  CjrlRobotDynamicsObjectConstructor<
  dynamicsJRLJapan::DynamicMultiBody, 
    dynamicsJRLJapan::HumanoidDynamicMultiBody, 
    dynamicsJRLJapan::JointFreeflyer, 
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> aRobotDynamicsObjectConstructor;
  
  CjrlHumanoidDynamicRobot * aHDR = aRobotDynamicsObjectConstructor.createhumanoidDynamicRobot();
  
  aHDMB = dynamic_cast<dynamicsJRLJapan::HumanoidDynamicMultiBody*>(aHDR);

  if (aHDMB==0)
    { 
      cerr<< "Dynamic cast on HDR failed " << endl;
      exit(-1);
    }
  aHDMB->parserVRML(aPath,aName,(char *)aMapFromJointToRank.c_str());
  aHDMB->SetHumanoidSpecificitiesFile(aSpecificitiesFileName);
  
  // Display tree of the joints.
  CjrlJoint* rootJoint = aHDMB->rootJoint();  

  // Tes the computation of the jacobian.
  double dInitPos[40] = { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  };

  int NbOfDofs = aHDMB->numberDof();
  std::cout << "NbOfDofs :" << NbOfDofs << std::endl;
  MAL_VECTOR_DIM(aCurrentConf,double,NbOfDofs);
  int lindex=0;
  for(int i=0;i<6;i++)
    aCurrentConf[lindex++] = 0.0;
  
  for(int i=0;i<(NbOfDofs-6 < 40 ? NbOfDofs-6 : 40) ;i++)
    aCurrentConf[lindex++] = dInitPos[i]*M_PI/180.0;
  //aCurrentConf[lindex++] = 0.0;
  
  aHDMB->currentConfiguration(aCurrentConf);

  MAL_VECTOR_DIM(aCurrentVel,double,NbOfDofs); 
  lindex=0;
  for(int i=0;i<NbOfDofs;i++)
    aCurrentVel[lindex++] = 0.0;
  
  aHDMB->currentVelocity(aCurrentVel);
  aHDMB->computeForwardKinematics();

  // Test the tree.
  RecursiveDisplayOfJoints(rootJoint);

  aHDMB->LinkBetweenJointsAndEndEffectorSemantic();

  std::vector<CjrlJoint *> aVec = aHDMB->jointVector();
  
  Joint  * aJoint = (Joint *)aVec[22]; // Try to get the hand.
  cout << aJoint->getName() << endl;  
  aJoint->computeJacobianJointWrtConfig();

  MAL_MATRIX(,double) aJ = aJoint->jacobianJointWrtConfig();
  
  //  DisplayMatrix(aJ);
  
  cout << "Root: " << ((Joint *)rootJoint)->getName() << endl;
  rootJoint->computeJacobianJointWrtConfig();
  aJ = rootJoint->jacobianJointWrtConfig();  
  cout << "Rank of Root: " << rootJoint->rankInConfiguration() << endl;

  //  DisplayMatrix(aJ);

  aJoint = (Joint *)aHDMB->waist();
  cout << "Name of the WAIST joint :" << endl;
  cout << aJoint->getName() << endl;

  //  aHDMB->computeJacobianCenterOfMass();
  //  cout << "Value of the CoM's Jacobian:" << endl
  //       << aHDMB->jacobianCenterOfMass() << endl;

  //  GoDownTree(aHDMB->rootJoint());

  cout << "Mass of the robot " << aHDMB->getMasse() << endl;
  cout << "Force " << aHDMB->getMasse()*9.81 << endl;

  // Test rank of the left hand.
  cout << "Rank of the left hand "<< endl;
  cout << aHDMB->leftWrist()->rankInConfiguration() << endl;
  cout << ((Joint *)aHDMB->leftWrist())->getName() << endl;
  cout << ((Joint *)aHDMB->leftWrist())->getIDinVRML() << endl;

  glutInitWindowSize(400, 400);
  glutInit(&argc, argv);

  /* Transperancy won't work properly without GLUT_ALPHA */
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
  glutCreateWindow("Display the wired 3D structure of the robot.");

  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutSpecialFunc(special);
  //  glutMouseFunc(processMouse);
  create_menu();

  myinit();

  glutReshapeFunc(myReshape);
  glutMainLoop();
  return 0;             /* ANSI C requires main to return int. */

  
  
}
