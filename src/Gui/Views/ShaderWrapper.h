#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>

namespace gui {

class ShaderWrapper : protected QOpenGLExtraFunctions{
public:
    ///
    /// Constructor reads each shader and builds the full shader pipeline.
    ///
    ShaderWrapper(
        const std::string& vertex_shader_path, 
        const std::string& fragment_shader_path
        );

    ///
    /// Use/activate the built shader.
    ///
    void Use();

    ///
    /// Access to the shader id.
    ///
    GLuint Id() const {
        return shader_id_;
    }

private:
    GLuint shader_id_;
};

} // namespace GUI
