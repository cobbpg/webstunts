// -*- c -*-

uniform mat4 worldProjection;
uniform mat3 normalProjection;

attribute vec3 vertexPosition;
attribute vec3 vertexNormal;
attribute vec3 vertexColour;
attribute vec3 vertexMaterialInfo;

varying vec3 fragmentPosition;
varying vec3 fragmentNormal;
varying vec3 fragmentColour;
varying vec3 fragmentMaterialInfo;
varying vec3 eyePosition;

void main(void) {
  vec4 p = vec4(vertexPosition, 1.0);
  vec4 e = worldProjection * p;
  fragmentPosition = vertexPosition;
  fragmentNormal = normalize(normalProjection * vertexNormal);
  fragmentColour = vertexColour;
  fragmentMaterialInfo = vertexMaterialInfo;
  eyePosition = e.xyz;
  gl_Position = e;
  gl_Position.z = -(e.z + (e.z + 0.5) * vertexMaterialInfo.r);
}
