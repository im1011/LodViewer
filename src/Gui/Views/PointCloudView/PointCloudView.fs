#version 100
precision mediump float;

uniform float alpha_;
varying vec4 v_rgba_;

void main() {
    gl_FragColor = vec4(v_rgba_[0] / 255.0, v_rgba_[1] / 255.0, v_rgba_[2] / 255.0, alpha_);
}
