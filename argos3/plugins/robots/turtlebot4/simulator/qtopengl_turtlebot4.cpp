/**
 * @file <turtlebot4/simulator/qtopengl_turtlebot4.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "qtopengl_turtlebot4.h"
#include "turtlebot4_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/turtlebot4/simulator/turtlebot4_measures.h>

namespace argos
{

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real WHEEL_DIAMETER = TURTLEBOT4_WHEEL_RADIUS * 2.0f;
   static const Real WHEEL_RADIUS = TURTLEBOT4_WHEEL_RADIUS;
   static const Real WHEEL_WIDTH = 0.02f; // Need this value
   static const Real HALF_WHEEL_WIDTH = WHEEL_WIDTH * 0.5f;
   static const Real INTERWHEEL_DISTANCE = 0.053f;
   static const Real HALF_INTERWHEEL_DISTANCE = TURTLEBOT4_HALF_WHEEL_DISTANCE;

   static const Real HALF_CHASSIS_WIDTH = TURTLEBOT4_HALF_WHEEL_DISTANCE - HALF_WHEEL_WIDTH;

   static const Real BODY_RADIUS = TURTLEBOT4_BASE_RADIUS;
   static const Real BODY_ELEVATION = TURTLEBOT4_BASE_ELEVATION; // to be checked!
   static const Real BODY_HEIGHT = TURTLEBOT4_BASE_HEIGHT;       // to be checked!
   static const Real LOWER_BODY_HEIGHT = TURTLEBOT4_LOWER_BODY_HEIGHT;

   static const Real LED_ELEVATION = TURTLEBOT4_LED_RING_ELEVATION;
   static const Real LED_DOT_RADIUS = UPPER_BODY_RADIUS * 0.5f;


   /****************************************/
   /****************************************/

   CQTOpenGLTurtlebot4::CQTOpenGLTurtlebot4() : m_unVertices(40)
   {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(4);

      /* Assign indices for better referencing (later) */
      m_unWheelList = m_unLists;
      m_unBodyList = m_unLists + 1;
      m_unUpperBodyList = m_unLists + 2;
      m_unColumnList = m_unLists + 3;

      /* Create the wheel display list */
      glNewList(m_unWheelList, GL_COMPILE);
      RenderWheel();
      glEndList();

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the upper body display list */
      glNewList(m_unUpperBodyList, GL_COMPILE);
      RenderUpperBody();
      glEndList();

      /* Create the column display list */
      glNewList(m_unColumnList, GL_COMPILE);
      RenderColumn();
      glEndList();

   }

   /****************************************/
   /****************************************/

   CQTOpenGLTurtlebot4::~CQTOpenGLTurtlebot4()
   {
      glDeleteLists(m_unLists, 4);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLTurtlebot4::Draw(CTurtlebot4Entity &c_entity)
   {
      /* Place the body */
      glCallList(m_unBodyList);

      /* Place the wheels */
      glPushMatrix();
      glTranslated(0.0f, HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      glPushMatrix();
      glTranslated(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();

      glCallList(m_unUpperBodyList);

      /* Columns (4 tower standoffs, positions from turtlebot4.urdf.xacro) */
      for (UInt32 i = 0; i < 4; ++i)
      {
         glPushMatrix();
         glTranslated(TURTLEBOT4_COLUMN_OFFSETS[i].GetX(),
                      TURTLEBOT4_COLUMN_OFFSETS[i].GetY(),
                      0.0f);
         glCallList(m_unColumnList);
         glPopMatrix();
      }

      /* Draw single LED at top plate center */
      CLEDEquippedEntity& cLEDEntity = c_entity.GetLEDEquippedEntity();
      const CColor& cColor = cLEDEntity.GetLED(0).GetColor();
      SetLEDMaterial(cColor.GetRed()   / 255.0f,
                     cColor.GetGreen() / 255.0f,
                     cColor.GetBlue()  / 255.0f);
      glPushMatrix();
      glTranslated(0.0f, 0.0f, LED_ELEVATION);
      CVector2 cV(LED_DOT_RADIUS, 0.0f);
      CRadians cStep(CRadians::TWO_PI / 10);
      glBegin(GL_POLYGON);
      glNormal3d(0.0f, 0.0f, 1.0f);
      for (UInt32 j = 0; j <= 10; ++j)
      {
         glVertex3d(cV.GetX(), cV.GetY(), 0.0f);
         cV.Rotate(cStep);
      }
      glEnd();
      glPopMatrix();
      /* Reset emission so it doesn't affect other objects */
      const GLfloat no_emission[] = {0.0f, 0.0f, 0.0f, 1.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_emission);
   }

   /* Base body — dark charcoal */
   void CQTOpenGLTurtlebot4::SetBaseMaterial()
   {
      const GLfloat ambient_diffuse[] = {0.2f, 0.2f, 0.2f, 1.0f};
      const GLfloat specular[] = {0.08f, 0.08f, 0.08f, 1.0f};
      const GLfloat shininess[] = {10.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
   }


   /* Upper deck — slightly lighter gray */
   void CQTOpenGLTurtlebot4::SetDeckMaterial()
   {
      const GLfloat ambient_diffuse[] = {0.18f, 0.18f, 0.18f, 1.0f};
      const GLfloat specular[] = {0.06f, 0.06f, 0.06f, 1.0f};
      const GLfloat shininess[] = {8.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
   }

   /* Columns — pure black matte */
   void CQTOpenGLTurtlebot4::SetColumnMaterial()
   {
      const GLfloat ambient_diffuse[] = {0.05f, 0.05f, 0.05f, 1.0f};
      const GLfloat specular[] = {0.02f, 0.02f, 0.02f, 1.0f};
      const GLfloat shininess[] = {2.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
   }

   /* LED — emissive colored dot */
   void CQTOpenGLTurtlebot4::SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue)
   {
      const GLfloat ambient_diffuse[] = {f_red, f_green, f_blue, 1.0f};
      const GLfloat emission[] = {f_red, f_green, f_blue, 1.0f};
      const GLfloat specular[] = {0.0f, 0.0f, 0.0f, 1.0f};
      const GLfloat shininess[] = {0.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
   }

   /* Wheels — black rubber, no shine */
   void CQTOpenGLTurtlebot4::SetWheelMaterial()
   {
      const GLfloat ambient_diffuse[] = {0.7f, 0.7f, 0.7f, 1.0f};
      const GLfloat specular[] = {0.00f, 0.00f, 0.00f, 1.0f};
      const GLfloat shininess[] = {1.0f};
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambient_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
   }

   void CQTOpenGLTurtlebot4::RenderWheel()
   {
      /* Right side */
      SetWheelMaterial();
      CVector2 cVertex(WHEEL_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector3 cNormal(-1.0f, -1.0f, 0.0f);
      cNormal.Normalize();
      glBegin(GL_POLYGON);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Left side */
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cNormal.Set(-1.0f, 1.0f, 0.0f);
      cNormal.Normalize();
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Tire */
      cNormal.Set(1.0f, 0.0f, 0.0f);
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_QUAD_STRIP);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3d(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         glVertex3d(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
   }

   void CQTOpenGLTurtlebot4::RenderUpperBody()
   {
      SetDeckMaterial();
      static const Real UPPER_BODY_THICKNESS = 0.01f;
      Real zBottom = BODY_ELEVATION + LOWER_BODY_HEIGHT + TURTLEBOT4_COLUMN_HEIGHT;
      Real zTop = zBottom + UPPER_BODY_THICKNESS;

      CVector2 cVertex(UPPER_BODY_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);

      /* Bottom disk */
      glBegin(GL_POLYGON);
      glNormal3d(0.0f, 0.0f, -1.0f);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3d(cVertex.GetX(), cVertex.GetY(), zBottom);
         cVertex.Rotate(cAngle);
      }
      glEnd();

      /* Side cylinder */
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(UPPER_BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), zTop);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), zBottom);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();

      /* Top disk */
      cVertex.Set(UPPER_BODY_RADIUS, 0.0f);
      glBegin(GL_POLYGON);
      glNormal3d(0.0f, 0.0f, 1.0f);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3d(cVertex.GetX(), cVertex.GetY(), zTop);
         cVertex.Rotate(cAngle);
      }
      glEnd();
   }

   void CQTOpenGLTurtlebot4::RenderBody()
   {
      SetBaseMaterial();
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);

      /* Bottom disk */
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      glBegin(GL_POLYGON);
      glNormal3d(0.0f, 0.0f, -1.0f);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(-cAngle);
      }
      glEnd();

      /* Side cylinder */
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + LOWER_BODY_HEIGHT);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();

      /* Top disk */
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_POLYGON);
      glNormal3d(0.0f, 0.0f, 1.0f);
      for (GLuint i = 0; i <= m_unVertices; i++)
      {
         glVertex3d(cVertex.GetX(), cVertex.GetY(),
                    BODY_ELEVATION + LOWER_BODY_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();
   }

   void CQTOpenGLTurtlebot4::RenderColumn()
   {
      SetColumnMaterial();
      Real baseZ = BODY_ELEVATION + LOWER_BODY_HEIGHT;
      Real topZ = baseZ + TURTLEBOT4_COLUMN_HEIGHT;

      CVector2 cNormal(1.0f, 0.0f);
      CVector2 cVertex(TURTLEBOT4_COLUMN_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);

      /* Side tube */
      glBegin(GL_QUAD_STRIP);
      for (UInt32 i = 0; i <= m_unVertices; ++i)
      {
         glNormal3d(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), baseZ);
         glVertex3d(cVertex.GetX(), cVertex.GetY(), topZ);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
   }

   class CQTOpenGLOperationDrawTurtlebot4Normal : public CQTOpenGLOperationDrawNormal
   {
   public:
      void ApplyTo(CQTOpenGLWidget &c_visualization,
                   CTurtlebot4Entity &c_entity)
      {
         static CQTOpenGLTurtlebot4 m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawTurtlebot4Selected : public CQTOpenGLOperationDrawSelected
   {
   public:
      void ApplyTo(CQTOpenGLWidget &c_visualization,
                   CTurtlebot4Entity &c_entity)
      {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawTurtlebot4Normal, CTurtlebot4Entity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawTurtlebot4Selected, CTurtlebot4Entity);
}