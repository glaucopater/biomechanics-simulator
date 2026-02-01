#include "biomechanics/OpenGLDebugDrawer.hpp"
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <cstdio>

namespace biomechanics {

OpenGLDebugDrawer::OpenGLDebugDrawer() = default;

void OpenGLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
  glColor3f(color.x(), color.y(), color.z());
  glVertex3f(from.x(), from.y(), from.z());
  glVertex3f(to.x(), to.y(), to.z());
}

void OpenGLDebugDrawer::drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                                        btScalar distance, int /*lifeTime*/, const btVector3& color) {
  glColor3f(color.x(), color.y(), color.z());
  glVertex3f(pointOnB.x(), pointOnB.y(), pointOnB.z());
  btVector3 end = pointOnB + normalOnB * distance;
  glVertex3f(end.x(), end.y(), end.z());
}

void OpenGLDebugDrawer::reportErrorWarning(const char* warningString) {
  std::fprintf(stderr, "Bullet: %s\n", warningString);
}

void OpenGLDebugDrawer::draw3dText(const btVector3& /*location*/, const char* /*textString*/) {
  // No-op: we don't render text in this minimal drawer
}

void OpenGLDebugDrawer::setDebugMode(int debugMode) {
  m_debugMode = debugMode;
}

int OpenGLDebugDrawer::getDebugMode() const {
  return m_debugMode;
}

}  // namespace biomechanics
