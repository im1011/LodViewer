#include "ShaderWrapper.h"

#include <fstream>
#include <sstream>
#include <iostream>

namespace gui {

ShaderWrapper::ShaderWrapper(
    const std::string& vertex_shader_path,
    const std::string& fragment_shader_path
    ) {

    std::string vert_shader_code;
    std::string frag_shader_code;
    std::ifstream vert_shader_file;
    std::ifstream frag_shader_file;

    // Ensure ifstream objects can throw exceptions:
    vert_shader_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    frag_shader_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    try{
        vert_shader_file.open(vertex_shader_path);
        frag_shader_file.open(fragment_shader_path);
        
        std::stringstream vert_shader_stream;
        std::stringstream frag_shader_stream;
        
        vert_shader_stream << vert_shader_file.rdbuf();
        frag_shader_stream << frag_shader_file.rdbuf();		
        
        vert_shader_file.close();
        frag_shader_file.close();
        
        vert_shader_code   = vert_shader_stream.str();
        frag_shader_code = frag_shader_stream.str();		
    } catch(std::ifstream::failure &e){
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ. Threw excception e: " << e.what() << std::endl;
    }

    const char* vert_shader_code_cstr = vert_shader_code.c_str();
    const char* frag_shader_code_cstr = frag_shader_code.c_str();

    // Initialize OpenGL functions to resolve to the current context. 
    // If context changes it has to be called again.
    initializeOpenGLFunctions();

    // Compile shaders
    int success;
    char info_log[512];

    // vertex Shader
    const GLuint vert_shader_id = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert_shader_id, 1, &vert_shader_code_cstr, nullptr);
    glCompileShader(vert_shader_id);
    
    glGetShaderiv(vert_shader_id, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(vert_shader_id, 512, nullptr, info_log);
        std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << info_log << std::endl;
    };

    // fragment Shader
    const GLuint frag_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag_shader_id, 1, &frag_shader_code_cstr, nullptr);
    glCompileShader(frag_shader_id);
    
    glGetShaderiv(frag_shader_id, GL_COMPILE_STATUS, &success);
    if(!success){
        glGetShaderInfoLog(frag_shader_id, 512, nullptr, info_log);
        std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << info_log << std::endl;
    };

    // Create overall shader program
    shader_id_ = glCreateProgram();
    glAttachShader(shader_id_, vert_shader_id);
    glAttachShader(shader_id_, frag_shader_id);
    glLinkProgram(shader_id_);
    
    glGetProgramiv(shader_id_, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(shader_id_, 512, nullptr, info_log);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << info_log << std::endl;
    }

    // Finally delete unneeded objects
    glDeleteShader(vert_shader_id);
    glDeleteShader(frag_shader_id);
}

void ShaderWrapper::Use(){
    glUseProgram(shader_id_);
}

} // namespace gui
