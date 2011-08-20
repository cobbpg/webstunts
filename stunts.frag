// -*- c -*-

#ifdef GL_ES
precision highp float;
#ifdef ANTI_MOIRE_ENABLED
#extension GL_OES_standard_derivatives : enable
#endif
#endif

uniform vec3 lightPosition;

varying vec3 fragmentPosition;
varying vec3 fragmentNormal;
varying vec3 fragmentColour;
varying vec3 fragmentMaterialInfo;
varying vec3 eyePosition;

#ifdef ANTI_MOIRE_ENABLED
float perturb(float x, float r) {
  float w = fwidth(x);

  return x + r * w * w * 2.0;
}
#endif

bool is_solid() {
#ifdef ANTI_MOIRE_ENABLED
  //float rand = texture2D(noiseSampler, gl_FragCoord.xy);
  float rand1 = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453) - 0.5;
  float rand2 = fract(sin(dot(gl_FragCoord.zx, vec2(15.9898, 74.233))) * 53758.5453) - 0.5;
  float rand3 = fract(sin(dot(gl_FragCoord.yz, vec2(18.9898, 71.233))) * 73758.5453) - 0.5;

  float prod1 = dot(fragmentPosition, vec3(2.0, 2.0, -2.0));
  float prod2 = dot(fragmentPosition, vec3(2.0, -2.0, 2.0));
  float prod3 = dot(fragmentPosition, vec3(-2.0, 2.0, 2.0));

  return
    fract(perturb(prod1, rand1)) < 0.2 ||
    fract(perturb(prod2, rand2)) < 0.2 ||
    fract(perturb(prod3, rand3)) < 0.2;
#else
  return
    fract(dot(fragmentPosition, vec3(2.0, 2.0, -2.0))) < 0.2 ||
    fract(dot(fragmentPosition, vec3(2.0, -2.0, 2.0))) < 0.2 ||
    fract(dot(fragmentPosition, vec3(-2.0, 2.0, 2.0))) < 0.2;
#endif
}

void main(void) {
  float surf = fragmentMaterialInfo.b;

  if (surf > 0.25 && (surf > 0.75 || is_solid())) {
    vec3 l = normalize(lightPosition - eyePosition);
    vec3 e = normalize(-eyePosition);
    vec3 n = fragmentNormal;
    vec3 r = normalize(reflect(-l, n));

    float lambert = max(dot(l, n), 0.0);
    float phong = max(dot(r, e), 0.0);
    float shininess = fragmentMaterialInfo.g;
    float intensity = 0.2 + lambert * 0.5 + pow(phong, shininess) * 0.3;
    float highlight = pow(phong, shininess) * min(1.0, shininess * 0.04);

    vec3 rgb = intensity * fragmentColour + highlight * vec3(1.0, 1.0, 1.0);
    //rgb = vec3(gl_FragCoord.z * 50.0);
    gl_FragColor = vec4(rgb, 1.0);
  } else {
    discard;
  }
}
