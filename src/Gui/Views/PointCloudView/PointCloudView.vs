#version 100

attribute vec4 xyz1_;
attribute vec4 rgba_;

uniform mat4 w2v_;
uniform mat4 proj_;
uniform float point_size_;

varying vec4 v_rgba_;

void main() {
    gl_PointSize = point_size_; 
    gl_Position = proj_ * w2v_ * xyz1_;
    v_rgba_ = rgba_;
}
