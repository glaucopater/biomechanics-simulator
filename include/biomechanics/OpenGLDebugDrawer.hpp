#pragma once

#include <btBulletDynamicsCommon.h>

namespace biomechanics {

/** OpenGL implementation of Bullet's btIDebugDraw for wireframe visualization. */
class OpenGLDebugDrawer : public btIDebugDraw {
public:
  OpenGLDebugDrawer();
  void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
  void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB, btScalar distance,
                       int lifeTime, const btVector3& color) override;
  void reportErrorWarning(const char* warningString) override;
  void draw3dText(const btVector3& location, const char* textString) override;
  void setDebugMode(int debugMode) override;
  int getDebugMode() const override;

private:
  int m_debugMode = DBG_DrawWireframe;
};

}  // namespace biomechanics
